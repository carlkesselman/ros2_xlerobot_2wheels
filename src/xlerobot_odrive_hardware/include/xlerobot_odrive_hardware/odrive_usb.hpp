// Native USB protocol for ODrive v3 boards on firmware 0.5.x (the ODESC on Carl's cart).
//
// The board describes itself: endpoint 0 returns a JSON tree of every property and function with
// its numeric id and type, and every later request carries a CRC16 of that JSON so a mismatched
// table is rejected instead of misread. This class fetches the JSON at open() and resolves
// properties by their dotted path ("axis0.controller.input_vel"), so it works with any 0.5.x
// build. (Factor-Robotics/odrive_ros2_control compiles one build's ids in; on this ODESC its CRC
// 0xebfb and ids are wrong - this board's table has CRC 0x9b40 and vbus_voltage at id 1, not 2.)
//
// Packet layout, little endian, one USB bulk transfer each way on interface 2 (EP 0x03 out,
// 0x83 in):
//   request:  seq u16 | endpoint id u16 (bit 15 = expect a reply) | reply length u16
//             | payload | trailer u16 (protocol version 1 for endpoint 0, else the JSON CRC)
//   reply:    seq u16 (echoed) | data
// A read is a request with an empty payload; a write sends the value as the payload; a function
// call with no arguments writes nothing to the function's id.
//
// No ROS here: odrive_probe and the tests use it on its own. Not thread safe; the hardware
// interface calls it from the controller manager's update thread only.

#pragma once

#include <cstdint>
#include <cstring>
#include <map>
#include <optional>
#include <string>
#include <vector>

struct libusb_context;
struct libusb_device_handle;

namespace xlerobot_odrive_hardware
{

// Firmware 0.5.x enum values (odrive/enums.py).
constexpr int32_t AXIS_STATE_IDLE = 1;
constexpr int32_t AXIS_STATE_CLOSED_LOOP_CONTROL = 8;
constexpr int32_t CONTROL_MODE_VELOCITY_CONTROL = 2;
constexpr int32_t INPUT_MODE_PASSTHROUGH = 1;
constexpr int32_t INPUT_MODE_VEL_RAMP = 2;

struct Endpoint
{
  uint16_t id = 0;
  std::string type;    // "float", "int32", "uint64", "bool", ... or "function"
};

// CRC16, polynomial 0x3d65, no reflection, as the firmware computes it over the JSON.
uint16_t crc16(const uint8_t * data, size_t len, uint16_t init);

// Flattens the endpoint JSON into path -> endpoint. Exposed for the tests.
std::map<std::string, Endpoint> parse_endpoints(const std::string & json_text);

class ODriveUSB
{
public:
  static constexpr uint16_t VENDOR_ID = 0x1209;
  static constexpr uint16_t PRODUCT_ID = 0x0d32;

  ODriveUSB() = default;
  ~ODriveUSB();
  ODriveUSB(const ODriveUSB &) = delete;
  ODriveUSB & operator=(const ODriveUSB &) = delete;

  // Opens the first board, or the one whose serial number (hex, as odrivetool prints it) matches,
  // and fetches its endpoint table. Returns false with error() set.
  bool open(const std::string & serial_hex = "", int timeout_ms = 20);
  void close();
  bool is_open() const {return handle_ != nullptr;}

  const std::string & error() const {return error_;}
  uint64_t serial_number() const {return serial_;}
  uint16_t json_crc() const {return json_crc_;}
  bool has(const std::string & path) const {return endpoints_.count(path) > 0;}

  // Typed property access by path. T must match the property's size; the type string is checked.
  template<typename T>
  bool read(const std::string & path, T & value)
  {
    const Endpoint * ep = find(path, sizeof(T));
    std::vector<uint8_t> reply;
    if (!ep || !transfer(ep->id, true, sizeof(T), {}, reply, json_crc_)) {return false;}
    if (reply.size() < sizeof(T)) {return fail(path + ": short reply");}
    std::memcpy(&value, reply.data(), sizeof(T));
    return true;
  }

  template<typename T>
  bool write(const std::string & path, const T & value)
  {
    const Endpoint * ep = find(path, sizeof(T));
    if (!ep) {return false;}
    std::vector<uint8_t> payload(sizeof(T));
    std::memcpy(payload.data(), &value, sizeof(T));
    std::vector<uint8_t> reply;
    // Ask for an (empty) acknowledgement, so a write that never reached the board is an error.
    return transfer(ep->id, true, 0, payload, reply, json_crc_);
  }

  // Calls a function that takes no arguments (clear_errors, watchdog_feed).
  bool call(const std::string & path);

private:
  const Endpoint * find(const std::string & path, size_t size);
  bool transfer(
    uint16_t endpoint_id, bool expect_reply, uint16_t reply_len,
    const std::vector<uint8_t> & payload, std::vector<uint8_t> & reply, uint16_t trailer);
  bool fetch_json();
  void release_handle();
  bool fail(const std::string & msg);

  libusb_context * ctx_ = nullptr;
  libusb_device_handle * handle_ = nullptr;
  int timeout_ms_ = 20;
  uint16_t seq_ = 0;
  uint16_t json_crc_ = 0;
  uint64_t serial_ = 0;
  std::map<std::string, Endpoint> endpoints_;
  std::string error_;
};

}  // namespace xlerobot_odrive_hardware
