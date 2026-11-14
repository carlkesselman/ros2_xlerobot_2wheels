#include "xlerobot_odrive_hardware/odrive_usb.hpp"

#include <libusb.h>

#include <strings.h>

#include <algorithm>
#include <cstdio>
#include <nlohmann/json.hpp>

namespace xlerobot_odrive_hardware
{

namespace
{
constexpr int INTERFACE = 2;
constexpr unsigned char EP_OUT = 0x03;
constexpr unsigned char EP_IN = 0x83;
constexpr int MAX_PACKET = 64;
constexpr uint16_t PROTOCOL_VERSION = 1;
constexpr uint16_t JSON_CHUNK = 32;   // bytes of the JSON per request, as odrivetool reads it

size_t type_size(const std::string & t)
{
  if (t == "bool" || t == "uint8" || t == "int8") {return 1;}
  if (t == "uint16" || t == "int16") {return 2;}
  if (t == "float" || t == "uint32" || t == "int32") {return 4;}
  if (t == "uint64" || t == "int64") {return 8;}
  return 0;
}

void walk(const nlohmann::json & nodes, const std::string & prefix, std::map<std::string, Endpoint> & out)
{
  for (const auto & n : nodes) {
    if (!n.contains("name")) {continue;}
    const std::string path = prefix + n["name"].get<std::string>();
    const std::string type = n.value("type", "");
    if (type == "object") {
      if (n.contains("members")) {walk(n["members"], path + ".", out);}
    } else if (n.contains("id")) {
      out[path] = Endpoint{n["id"].get<uint16_t>(), type};
    }
  }
}
}  // namespace

uint16_t crc16(const uint8_t * data, size_t len, uint16_t init)
{
  uint16_t c = init;
  for (size_t i = 0; i < len; ++i) {
    c ^= static_cast<uint16_t>(data[i]) << 8;
    for (int b = 0; b < 8; ++b) {
      c = (c & 0x8000) ? static_cast<uint16_t>((c << 1) ^ 0x3d65) : static_cast<uint16_t>(c << 1);
    }
  }
  return c;
}

std::map<std::string, Endpoint> parse_endpoints(const std::string & json_text)
{
  std::map<std::string, Endpoint> out;
  walk(nlohmann::json::parse(json_text), "", out);
  return out;
}

ODriveUSB::~ODriveUSB() {close();}

bool ODriveUSB::fail(const std::string & msg)
{
  error_ = msg;
  return false;
}

bool ODriveUSB::open(const std::string & serial_hex, int timeout_ms)
{
  close();
  timeout_ms_ = timeout_ms;
  int rc = libusb_init(&ctx_);
  if (rc != LIBUSB_SUCCESS) {
    ctx_ = nullptr;
    return fail(std::string("libusb_init: ") + libusb_error_name(rc));
  }

  libusb_device ** list = nullptr;
  ssize_t n = libusb_get_device_list(ctx_, &list);
  std::string seen;
  for (ssize_t i = 0; i < n && !handle_; ++i) {
    libusb_device_descriptor d{};
    if (libusb_get_device_descriptor(list[i], &d) != LIBUSB_SUCCESS) {continue;}
    if (d.idVendor != VENDOR_ID || d.idProduct != PRODUCT_ID) {continue;}
    libusb_device_handle * h = nullptr;
    rc = libusb_open(list[i], &h);
    if (rc != LIBUSB_SUCCESS) {
      seen += std::string(" (open failed: ") + libusb_error_name(rc) +
        ", check the udev rule for 1209:0d32)";
      continue;
    }
    libusb_set_auto_detach_kernel_driver(h, 1);   // Linux: the CDC driver holds the other interfaces
    rc = libusb_claim_interface(h, INTERFACE);
    if (rc != LIBUSB_SUCCESS) {
      seen += std::string(" (claim failed: ") + libusb_error_name(rc) + ", is another program using it?)";
      libusb_close(h);
      continue;
    }
    handle_ = h;
    // Stale replies from an earlier client would desynchronise the sequence numbers: drain them.
    unsigned char junk[MAX_PACKET];
    int got = 0;
    while (libusb_bulk_transfer(handle_, EP_IN, junk, MAX_PACKET, &got, 5) == LIBUSB_SUCCESS) {}

    if (!fetch_json() || !read("serial_number", serial_)) {
      seen += " (" + error_ + ")";
      release_handle();
      continue;
    }
    if (!serial_hex.empty()) {
      char buf[32];
      std::snprintf(buf, sizeof(buf), "%llX", static_cast<unsigned long long>(serial_));
      if (strcasecmp(buf, serial_hex.c_str()) != 0) {
        seen += std::string(" (") + buf + " is not the requested " + serial_hex + ")";
        release_handle();
        continue;
      }
    }
  }
  if (list) {libusb_free_device_list(list, 1);}
  if (!handle_) {
    close();
    return fail("no ODrive 1209:0d32 found" + seen);
  }
  return true;
}

void ODriveUSB::release_handle()
{
  if (handle_) {
    libusb_release_interface(handle_, INTERFACE);
    libusb_close(handle_);
    handle_ = nullptr;
  }
  endpoints_.clear();
}

void ODriveUSB::close()
{
  release_handle();
  if (ctx_) {
    libusb_exit(ctx_);
    ctx_ = nullptr;
  }
}

bool ODriveUSB::transfer(
  uint16_t endpoint_id, bool expect_reply, uint16_t reply_len,
  const std::vector<uint8_t> & payload, std::vector<uint8_t> & reply, uint16_t trailer)
{
  if (!handle_) {return fail("not open");}
  if (payload.size() + 8 > MAX_PACKET) {return fail("payload too long");}

  seq_ = static_cast<uint16_t>((seq_ + 1) & 0x7fff);
  const uint16_t seq = seq_ | 0x80;   // the scheme odrivetool and Factor's driver use
  const uint16_t ep = endpoint_id | (expect_reply ? 0x8000 : 0);

  unsigned char pkt[MAX_PACKET];
  size_t k = 0;
  auto put16 = [&](uint16_t v) {pkt[k++] = v & 0xff; pkt[k++] = v >> 8;};
  put16(seq);
  put16(ep);
  put16(reply_len);
  for (uint8_t b : payload) {pkt[k++] = b;}
  put16(trailer);

  int sent = 0;
  int rc = libusb_bulk_transfer(handle_, EP_OUT, pkt, static_cast<int>(k), &sent, timeout_ms_);
  if (rc != LIBUSB_SUCCESS || sent != static_cast<int>(k)) {
    return fail(std::string("usb write: ") + libusb_error_name(rc));
  }
  if (!expect_reply) {return true;}

  // A reply to an earlier request that timed out can still be queued; skip up to two of them.
  for (int attempt = 0; attempt < 3; ++attempt) {
    unsigned char in[MAX_PACKET];
    int got = 0;
    rc = libusb_bulk_transfer(handle_, EP_IN, in, MAX_PACKET, &got, timeout_ms_);
    if (rc != LIBUSB_SUCCESS) {return fail(std::string("usb read: ") + libusb_error_name(rc));}
    if (got < 2) {continue;}
    const uint16_t rseq = static_cast<uint16_t>(in[0] | (in[1] << 8));
    if ((rseq & 0x7fff) != (seq & 0x7fff)) {continue;}
    reply.assign(in + 2, in + got);
    return true;
  }
  return fail("usb read: sequence mismatch");
}

bool ODriveUSB::fetch_json()
{
  std::string blob;
  for (;;) {
    const uint32_t offset = static_cast<uint32_t>(blob.size());
    std::vector<uint8_t> payload(4);
    std::memcpy(payload.data(), &offset, 4);
    std::vector<uint8_t> chunk;
    const int saved = timeout_ms_;
    timeout_ms_ = std::max(timeout_ms_, 200);
    const bool ok = transfer(0, true, JSON_CHUNK, payload, chunk, PROTOCOL_VERSION);
    timeout_ms_ = saved;
    if (!ok) {return fail("reading the endpoint table: " + error_);}
    if (chunk.empty()) {break;}
    blob.append(chunk.begin(), chunk.end());
    if (blob.size() > 1 << 20) {return fail("endpoint table larger than 1 MB");}
  }
  json_crc_ = crc16(reinterpret_cast<const uint8_t *>(blob.data()), blob.size(), PROTOCOL_VERSION);
  try {
    endpoints_ = parse_endpoints(blob);
  } catch (const std::exception & e) {
    return fail(std::string("endpoint table is not JSON: ") + e.what());
  }
  if (endpoints_.empty()) {return fail("empty endpoint table");}
  return true;
}

const Endpoint * ODriveUSB::find(const std::string & path, size_t size)
{
  auto it = endpoints_.find(path);
  if (it == endpoints_.end()) {
    fail(path + ": no such property on this firmware");
    return nullptr;
  }
  if (size && type_size(it->second.type) != size) {
    fail(path + ": is " + it->second.type + ", not " + std::to_string(size) + " bytes");
    return nullptr;
  }
  return &it->second;
}

bool ODriveUSB::call(const std::string & path)
{
  const Endpoint * ep = find(path, 0);
  if (!ep) {return false;}
  if (ep->type != "function") {return fail(path + ": not a function");}
  std::vector<uint8_t> reply;
  return transfer(ep->id, true, 0, {}, reply, json_crc_);
}

}  // namespace xlerobot_odrive_hardware
