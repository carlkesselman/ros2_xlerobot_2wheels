// Read-only check of the ODESC from a shell, with the stack stopped (only one program can hold the
// board's USB interface):
//
//   ros2 run xlerobot_odrive_hardware odrive_probe [serial_hex]
//
// Prints firmware, bus voltage, the endpoint table's CRC, and per axis the state, error words,
// position and velocity. Commands nothing.

#include <cstdio>
#include <string>

#include "xlerobot_odrive_hardware/odrive_usb.hpp"

using xlerobot_odrive_hardware::ODriveUSB;

int main(int argc, char ** argv)
{
  ODriveUSB od;
  if (!od.open(argc > 1 ? argv[1] : "", 200)) {
    std::fprintf(stderr, "%s\n", od.error().c_str());
    return 1;
  }
  uint8_t maj = 0, min = 0, rev = 0;
  float vbus = 0;
  bool brake = true;
  od.read("fw_version_major", maj);
  od.read("fw_version_minor", min);
  od.read("fw_version_revision", rev);
  od.read("vbus_voltage", vbus);
  od.read("brake_resistor_armed", brake);
  std::printf("ODrive %llX  firmware %u.%u.%u  table CRC 0x%04x  vbus %.2f V  brake resistor %s\n",
    static_cast<unsigned long long>(od.serial_number()), maj, min, rev, od.json_crc(), vbus,
    brake ? "armed" : "DISARMED");
  for (const char * ax : {"axis0", "axis1"}) {
    const std::string a(ax);
    int32_t state = 0, err = 0, merr = 0, eerr = 0, cerr = 0;
    float pos = 0, vel = 0;
    bool ok = od.read(a + ".current_state", state) && od.read(a + ".error", err) &&
      od.read(a + ".motor.error", merr) && od.read(a + ".encoder.error", eerr) &&
      od.read(a + ".controller.error", cerr) && od.read(a + ".encoder.pos_estimate", pos) &&
      od.read(a + ".encoder.vel_estimate", vel);
    if (!ok) {
      std::printf("%s: %s\n", ax, od.error().c_str());
      continue;
    }
    std::printf("%s  state %d  errors axis 0x%x motor 0x%x encoder 0x%x controller 0x%x  "
      "pos %.3f turns  vel %.3f turns/s\n", ax, state, err, merr, eerr, cerr, pos, vel);
  }
  return 0;
}
