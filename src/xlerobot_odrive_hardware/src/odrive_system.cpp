#include "xlerobot_odrive_hardware/odrive_system.hpp"

#include <cmath>
#include <sstream>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace xlerobot_odrive_hardware
{

namespace
{
constexpr double TWO_PI = 2.0 * M_PI;
constexpr double FF_FADE_TPS = 0.03;         // below this wheel speed friction feed-forward fades out
constexpr double STATE_CHECK_S = 0.2;        // how often read() looks at the axis states
constexpr double REARM_INTERVAL_S = 2.0;
constexpr double FEED_INTERVAL_S = 0.05;
constexpr int MAX_USB_FAILURES = 5;          // consecutive, before read/write report ERROR

double param(const std::unordered_map<std::string, std::string> & m, const std::string & k, double dflt)
{
  auto it = m.find(k);
  return (it == m.end() || it->second.empty()) ? dflt : std::stod(it->second);
}

// What an axis error word most likely means, fw 0.5.x bits (odrive_base.py's _explain).
std::string explain(uint32_t bits)
{
  if (bits & 0x30) {
    return "BRAKE_RESISTOR / MOTOR_DISARMED: a latched DC-bus fault (over-voltage, regen during an abrupt "
           "stop or an impact); only a power cycle of the ODESC clears it";
  }
  if (bits & 0x6) {return "DC bus under/over voltage: check the pack and the XH-M609";}
  if (bits & 0x800) {return "WATCHDOG_TIMER_EXPIRED: nothing fed the watchdog in time";}
  if (bits & 0x100) {return "ENCODER_FAILED: Hall wiring, or the encoder needs calibrating";}
  if (bits & 0x40) {return "MOTOR_FAILED: a phase connection, or the motor needs calibrating";}
  if (bits & 0x200) {return "CONTROLLER_FAILED (e.g. OVERSPEED)";}
  return "see XLeRobot's config/odrive-check.sh";
}
}  // namespace

ODriveSystem::CallbackReturn ODriveSystem::on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {return CallbackReturn::ERROR;}
  const auto & hw = info_.hardware_parameters;
  try {
    serial_hex_ = hw.count("serial_number") ? hw.at("serial_number") : "";
    max_wheel_accel_ = param(hw, "max_wheel_accel", 0.0);
    friction_ff_ = param(hw, "friction_ff_nm", 0.0);
    inertia_ff_ = param(hw, "inertia_ff", 0.0);
    watchdog_s_ = param(hw, "watchdog_s", 0.5);
    usb_timeout_ms_ = static_cast<int>(param(hw, "usb_timeout_ms", 20));
  } catch (const std::exception & e) {
    RCLCPP_FATAL(get_logger(), "bad hardware parameter: %s", e.what());
    return CallbackReturn::ERROR;
  }
  feed_forward_ = friction_ff_ > 0.0 || inertia_ff_ > 0.0;

  for (const auto & j : info_.joints) {
    if (j.command_interfaces.size() != 1 || j.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(get_logger(), "%s: needs exactly one velocity command interface", j.name.c_str());
      return CallbackReturn::ERROR;
    }
    for (const auto & s : j.state_interfaces) {
      if (s.name != hardware_interface::HW_IF_POSITION && s.name != hardware_interface::HW_IF_VELOCITY) {
        RCLCPP_FATAL(get_logger(), "%s: state interface '%s' not supported", j.name.c_str(), s.name.c_str());
        return CallbackReturn::ERROR;
      }
    }
    auto a = j.parameters.find("axis");
    if (a == j.parameters.end() || (a->second != "0" && a->second != "1")) {
      RCLCPP_FATAL(get_logger(), "%s: <param name=\"axis\"> must be 0 or 1", j.name.c_str());
      return CallbackReturn::ERROR;
    }
    Wheel w;
    w.joint = j.name;
    w.ax = "axis" + a->second;
    const auto & ci = j.command_interfaces[0];
    w.vmax = ci.max.empty() ? 1e9 : std::fabs(std::stod(ci.max));
    wheels_.push_back(w);
  }
  if (wheels_.size() == 2 && wheels_[0].ax == wheels_[1].ax) {
    RCLCPP_FATAL(get_logger(), "both joints are on %s", wheels_[0].ax.c_str());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_logger(), "%zu wheel(s); accel %.2f rad/s^2, feed-forward %s (%.2f Nm, %.3f Nm/(turn/s^2)), "
    "watchdog %.2f s", wheels_.size(), max_wheel_accel_, feed_forward_ ? "on" : "off", friction_ff_,
    inertia_ff_, watchdog_s_);
  return CallbackReturn::SUCCESS;
}

bool ODriveSystem::usb_ok(bool ok, const char * what)
{
  if (ok) {
    usb_failures_ = 0;
    return true;
  }
  ++usb_failures_;
  RCLCPP_WARN_THROTTLE(get_logger(), steady_, 1000, "%s: %s", what, usb_.error().c_str());
  return false;
}

void ODriveSystem::log_held_errors(const char * when)
{
  std::ostringstream s;
  uint32_t bits = 0;
  for (const auto & w : wheels_) {
    int32_t axis = 0, motor = 0, encoder = 0, controller = 0;
    usb_.read(p(w, "error"), axis);
    usb_.read(p(w, "motor.error"), motor);
    usb_.read(p(w, "encoder.error"), encoder);
    usb_.read(p(w, "controller.error"), controller);
    if (axis || motor || encoder || controller) {
      s << " " << w.joint << " (" << w.ax << "): axis 0x" << std::hex << axis << " motor 0x" << motor
        << " encoder 0x" << encoder << " controller 0x" << controller << std::dec << ";";
      bits |= static_cast<uint32_t>(axis);
    }
  }
  if (!s.str().empty()) {
    RCLCPP_WARN(get_logger(), "ODrive errors %s:%s %s", when, s.str().c_str(), explain(bits).c_str());
  }
}

ODriveSystem::CallbackReturn ODriveSystem::on_configure(const rclcpp_lifecycle::State &)
{
  if (!usb_.open(serial_hex_, usb_timeout_ms_)) {
    RCLCPP_ERROR(get_logger(), "%s", usb_.error().c_str());
    return CallbackReturn::ERROR;
  }
  uint8_t maj = 0, min = 0, rev = 0;
  float vbus = 0.0f;
  usb_.read("fw_version_major", maj);
  usb_.read("fw_version_minor", min);
  usb_.read("fw_version_revision", rev);
  usb_.read("vbus_voltage", vbus);
  RCLCPP_INFO(get_logger(), "ODrive %llX, firmware %u.%u.%u, endpoint table CRC 0x%04x, vbus %.1f V",
    static_cast<unsigned long long>(usb_.serial_number()), maj, min, rev, usb_.json_crc(), vbus);
  if (maj != 0 || min != 5) {
    RCLCPP_ERROR(get_logger(), "written for firmware 0.5.x; 0.6 renamed the encoder and controller properties");
    usb_.close();
    return CallbackReturn::ERROR;
  }

  // Record what the board is holding before anything clears it.
  log_held_errors("held from before this start");

  bool ok = true;
  for (auto & w : wheels_) {
    // The board's pos_estimate carries over from whatever ran before, and diff_drive_controller
    // would count the jump from the state's initial 0 as travel. Report positions from here.
    float pos = 0.0f;
    ok &= usb_.read(p(w, "encoder.pos_estimate"), pos);
    w.pos_offset = pos;
    ok &= usb_.write(p(w, "config.enable_watchdog"), false);
    ok &= usb_.write(p(w, "controller.config.control_mode"), CONTROL_MODE_VELOCITY_CONTROL);
    if (feed_forward_ || max_wheel_accel_ <= 0.0) {
      ok &= usb_.write(p(w, "controller.config.input_mode"), INPUT_MODE_PASSTHROUGH);
      ok &= usb_.write(p(w, "controller.input_torque"), 0.0f);
    } else {
      ok &= usb_.write(p(w, "controller.config.vel_ramp_rate"), static_cast<float>(max_wheel_accel_ / TWO_PI));
      ok &= usb_.write(p(w, "controller.config.input_mode"), INPUT_MODE_VEL_RAMP);
    }
    ok &= usb_.write(p(w, "controller.input_vel"), 0.0f);
  }
  if (!ok) {
    RCLCPP_ERROR(get_logger(), "configuring the axes: %s", usb_.error().c_str());
    usb_.close();
    return CallbackReturn::ERROR;
  }
  for (const auto & w : wheels_) {
    set_command(w.joint + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);
  }
  return CallbackReturn::SUCCESS;
}

bool ODriveSystem::enter_closed_loop(bool settle)
{
  for (const auto & w : wheels_) {
    usb_.call(p(w, "clear_errors"));
    if (watchdog_armed_) {usb_.call(p(w, "watchdog_feed"));}
  }
  if (settle) {std::this_thread::sleep_for(std::chrono::milliseconds(200));}
  bool ok = true;
  for (const auto & w : wheels_) {
    ok &= usb_.write(p(w, "requested_state"), AXIS_STATE_CLOSED_LOOP_CONTROL);
  }
  if (settle) {std::this_thread::sleep_for(std::chrono::milliseconds(500));}
  return ok;
}

ODriveSystem::CallbackReturn ODriveSystem::on_activate(const rclcpp_lifecycle::State &)
{
  bool armed = true;
  if (usb_.has("brake_resistor_armed") && usb_.read("brake_resistor_armed", armed) && !armed) {
    RCLCPP_ERROR(get_logger(), "ODrive brake resistor is DISARMED; power-cycle the ODESC");
    return CallbackReturn::ERROR;
  }
  for (auto & w : wheels_) {
    w.setpoint = 0.0;
    usb_.write(p(w, "controller.input_vel"), 0.0f);
    set_command(w.joint + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);
  }
  if (!enter_closed_loop(true)) {
    RCLCPP_ERROR(get_logger(), "requesting closed loop: %s", usb_.error().c_str());
    return CallbackReturn::ERROR;
  }
  for (const auto & w : wheels_) {
    int32_t state = 0, err = 0;
    usb_.read(p(w, "current_state"), state);
    usb_.read(p(w, "error"), err);
    if (state != AXIS_STATE_CLOSED_LOOP_CONTROL || err) {
      log_held_errors("entering closed loop");
      RCLCPP_ERROR(get_logger(), "%s did not enter closed loop (state %d)", w.joint.c_str(), state);
      idle_and_disarm();
      return CallbackReturn::ERROR;
    }
  }
  if (watchdog_s_ > 0.0) {
    for (const auto & w : wheels_) {
      usb_.write(p(w, "config.watchdog_timeout"), static_cast<float>(watchdog_s_));
      usb_.call(p(w, "watchdog_feed"));
      usb_.write(p(w, "config.enable_watchdog"), true);
    }
    watchdog_armed_ = true;
    last_feed_ = steady_.now();
    RCLCPP_INFO(get_logger(), "watchdog armed: %.2f s without a write and the wheels coast", watchdog_s_);
  }
  usb_failures_ = 0;
  RCLCPP_INFO(get_logger(), "wheels in closed-loop velocity control");
  return CallbackReturn::SUCCESS;
}

void ODriveSystem::idle_and_disarm()
{
  if (!usb_.is_open()) {return;}
  for (auto & w : wheels_) {
    if (feed_forward_) {usb_.write(p(w, "controller.input_torque"), 0.0f);}
    usb_.write(p(w, "controller.input_vel"), 0.0f);
    w.setpoint = 0.0;
    // leave the board as other tools (odrive-check, lerobot) expect it: no watchdog
    usb_.write(p(w, "config.enable_watchdog"), false);
    usb_.write(p(w, "requested_state"), AXIS_STATE_IDLE);
  }
  watchdog_armed_ = false;
}

ODriveSystem::CallbackReturn ODriveSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  idle_and_disarm();
  RCLCPP_INFO(get_logger(), "wheels idle");
  return CallbackReturn::SUCCESS;
}

ODriveSystem::CallbackReturn ODriveSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  idle_and_disarm();
  usb_.close();
  return CallbackReturn::SUCCESS;
}

ODriveSystem::CallbackReturn ODriveSystem::on_shutdown(const rclcpp_lifecycle::State &)
{
  idle_and_disarm();
  usb_.close();
  return CallbackReturn::SUCCESS;
}

ODriveSystem::CallbackReturn ODriveSystem::on_error(const rclcpp_lifecycle::State &)
{
  idle_and_disarm();   // best effort; if USB is gone the board's watchdog stops the wheels
  usb_.close();
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type ODriveSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (const auto & w : wheels_) {
    float pos = 0.0f, vel = 0.0f;
    const bool ok = usb_ok(usb_.read(p(w, "encoder.pos_estimate"), pos), "reading position") &&
      usb_ok(usb_.read(p(w, "encoder.vel_estimate"), vel), "reading velocity");
    if (ok) {
      set_state(w.joint + "/" + hardware_interface::HW_IF_POSITION, (pos - w.pos_offset) * TWO_PI);
      set_state(w.joint + "/" + hardware_interface::HW_IF_VELOCITY, vel * TWO_PI);
    }
  }

  // A board that rebooted (DC bus lost and back) or faulted sits in IDLE and ignores input_vel
  // without an error anywhere. Look a few times a second; re-arm at most every REARM_INTERVAL_S.
  const auto now = steady_.now();
  if ((now - last_state_check_).seconds() >= STATE_CHECK_S) {
    last_state_check_ = now;
    bool all_closed = true;
    std::ostringstream states;
    for (const auto & w : wheels_) {
      int32_t st = 0;
      if (!usb_ok(usb_.read(p(w, "current_state"), st), "reading axis state")) {continue;}
      states << " " << w.joint << "=" << st;
      all_closed &= st == AXIS_STATE_CLOSED_LOOP_CONTROL;
    }
    if (!all_closed && (now - last_rearm_).seconds() >= REARM_INTERVAL_S) {
      last_rearm_ = now;
      ++rearms_;
      float vbus = NAN;
      usb_.read("vbus_voltage", vbus);
      RCLCPP_WARN(get_logger(), "axes out of closed loop (states%s, vbus %.1f V), re-arming (#%d)",
        states.str().c_str(), vbus, rearms_);
      log_held_errors("before re-arm");
      bool br = true;
      if (usb_.has("brake_resistor_armed") && usb_.read("brake_resistor_armed", br) && !br) {
        RCLCPP_ERROR(get_logger(), "brake resistor DISARMED: re-arming cannot work until the ODESC is power-cycled");
      }
      for (auto & w : wheels_) {w.setpoint = 0.0;}   // restart the software ramp from rest
      enter_closed_loop(false);
    }
  }
  return usb_failures_ >= MAX_USB_FAILURES ? hardware_interface::return_type::ERROR :
         hardware_interface::return_type::OK;
}

hardware_interface::return_type ODriveSystem::write(const rclcpp::Time &, const rclcpp::Duration & period)
{
  const double dt = std::min(std::max(period.seconds(), 0.0), 0.1);
  for (auto & w : wheels_) {
    double cmd = get_command<double>(w.joint + "/" + hardware_interface::HW_IF_VELOCITY);
    if (!std::isfinite(cmd)) {cmd = 0.0;}
    cmd = std::clamp(cmd, -w.vmax, w.vmax);
    const double target = cmd / TWO_PI;   // turns/s

    if (feed_forward_) {
      // Software ramp + feed-forward torque, as odrive_base.py's _send_ff.
      double step = target - w.setpoint;
      double accel = 0.0;
      if (max_wheel_accel_ > 0.0) {
        const double a_max = max_wheel_accel_ / TWO_PI;
        step = std::clamp(step, -a_max * dt, a_max * dt);
      }
      if (dt > 0.0) {accel = step / dt;}
      w.setpoint += step;
      const double torque = inertia_ff_ * accel + friction_ff_ * std::tanh(w.setpoint / FF_FADE_TPS);
      usb_ok(usb_.write(p(w, "controller.input_vel"), static_cast<float>(w.setpoint)), "writing velocity");
      usb_ok(usb_.write(p(w, "controller.input_torque"), static_cast<float>(torque)), "writing torque");
    } else {
      usb_ok(usb_.write(p(w, "controller.input_vel"), static_cast<float>(target)), "writing velocity");
    }
  }

  if (watchdog_armed_) {
    const auto now = steady_.now();
    if ((now - last_feed_).seconds() >= FEED_INTERVAL_S) {
      for (const auto & w : wheels_) {
        usb_ok(usb_.call(p(w, "watchdog_feed")), "feeding watchdog");
      }
      last_feed_ = now;
    }
  }
  return usb_failures_ >= MAX_USB_FAILURES ? hardware_interface::return_type::ERROR :
         hardware_interface::return_type::OK;
}

}  // namespace xlerobot_odrive_hardware

PLUGINLIB_EXPORT_CLASS(xlerobot_odrive_hardware::ODriveSystem, hardware_interface::SystemInterface)
