#include <gtest/gtest.h>

#include <string>

#include "xlerobot_odrive_hardware/odrive_usb.hpp"

using namespace xlerobot_odrive_hardware;

TEST(Crc16, EmptyIsInit)
{
  EXPECT_EQ(crc16(nullptr, 0, 1), 1);
}

TEST(Crc16, MatchesReference)
{
  // Reference value from the Python implementation that the ODESC accepted (odrive_probe.py).
  const std::string s = "[{\"name\":\"vbus_voltage\",\"id\":1,\"type\":\"float\",\"access\":\"r\"}]";
  EXPECT_EQ(crc16(reinterpret_cast<const uint8_t *>(s.data()), s.size(), 1), 0x35c9);
}

TEST(Endpoints, FlattensNestedObjects)
{
  const std::string j = R"([
    {"name":"vbus_voltage","id":1,"type":"float","access":"r"},
    {"name":"axis0","type":"object","members":[
      {"name":"error","id":71,"type":"int32","access":"rw"},
      {"name":"controller","type":"object","members":[
        {"name":"input_vel","id":195,"type":"float","access":"rw"}]},
      {"name":"watchdog_feed","id":298,"type":"function","inputs":[],"outputs":[]}]}])";
  auto m = parse_endpoints(j);
  ASSERT_EQ(m.size(), 4u);
  EXPECT_EQ(m.at("vbus_voltage").id, 1);
  EXPECT_EQ(m.at("axis0.error").type, "int32");
  EXPECT_EQ(m.at("axis0.controller.input_vel").id, 195);
  EXPECT_EQ(m.at("axis0.watchdog_feed").type, "function");
}
