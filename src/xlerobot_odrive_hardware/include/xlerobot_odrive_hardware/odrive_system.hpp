// ros2_control SystemInterface for the cart's two hub wheels on one ODESC (ODrive v3, fw 0.5.x).
//
// Joints: velocity command (rad/s), position (rad) and velocity (rad/s) state, each with an
// <param name="axis"> naming the board axis. The joints report the raw axis: the mirrored left
// motor's sign is diff_drive_controller's left_wheel_radius_multiplier, applied nowhere here.
//
// What it carries over from XLeRobot's odrive_base.py, which ran this cart under lerobot:
//  - velocity control, the acceleration limit either on the board (INPUT_MODE_VEL_RAMP) or, when
//    feed-forward is on, as a software ramp with input_torque each cycle (the board's ramp would
//    overwrite input_torque)
//  - the board's watchdog: armed on activate, fed from write(); if the controller manager stalls
//    or dies the axes drop to IDLE and the wheels coast
//  - error words logged before anything clears them (fw 0.5.1 keeps no error log), and a
//    disarmed brake resistor refuses activation
//  - re-arming: a board that browned out or faulted boots to IDLE and silently ignores input_vel;
//    read() checks the axis states a few times a second and requests closed loop again, at most
//    every 2 s, with a log line each time
//
// Hardware parameters (all optional):
//   serial_number     hex, as odrivetool prints it; empty = the first board
//   max_wheel_accel   rad/s^2 per wheel; 0 = none
//   friction_ff_nm    Nm, Coulomb friction feed-forward (fades out below ~0.03 turn/s)
//   inertia_ff        Nm per turn/s^2
//   watchdog_s        board watchdog timeout; 0 = off
//   usb_timeout_ms    per USB transfer in the control loop (default 20)

#pragma once

#include <array>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "xlerobot_odrive_hardware/odrive_usb.hpp"

namespace xlerobot_odrive_hardware
{

class ODriveSystem : public hardware_interface::SystemInterface
{
public:
  using CallbackReturn = hardware_interface::CallbackReturn;

  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct Wheel
  {
    std::string joint;
    std::string ax;           // "axis0" / "axis1"
    double vmax = 0.0;        // rad/s, from the command interface's max
    double setpoint = 0.0;    // turns/s, software ramp state (feed-forward mode)
    double pos_offset = 0.0;  // turns, pos_estimate at configure, reported as 0 rad
  };

  std::string p(const Wheel & w, const char * rest) const {return w.ax + "." + rest;}
  bool enter_closed_loop(bool settle);
  void log_held_errors(const char * when);
  void idle_and_disarm();
  bool usb_ok(bool ok, const char * what);

  ODriveUSB usb_;
  std::vector<Wheel> wheels_;
  std::string serial_hex_;
  double max_wheel_accel_ = 0.0;   // rad/s^2
  double friction_ff_ = 0.0;
  double inertia_ff_ = 0.0;
  double watchdog_s_ = 0.5;
  int usb_timeout_ms_ = 20;
  bool feed_forward_ = false;

  bool watchdog_armed_ = false;
  rclcpp::Time last_feed_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time last_state_check_{0, 0, RCL_STEADY_TIME};
  rclcpp::Time last_rearm_{0, 0, RCL_STEADY_TIME};
  int rearms_ = 0;
  int usb_failures_ = 0;
  rclcpp::Clock steady_{RCL_STEADY_TIME};
};

}  // namespace xlerobot_odrive_hardware
