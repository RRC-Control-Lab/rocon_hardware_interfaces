// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RMD_HARDWARE_INTERFACE__RMD_HARDWARE_INTERFACE_HPP_
#define RMD_HARDWARE_INTERFACE__RMD_HARDWARE_INTERFACE_HPP_

#include <linux/can.h>
#include <linux/can/raw.h>

#include <string>
#include <unordered_map>
#include <vector>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <array>
#include <algorithm>
#include <cmath> 

#include "rmd_hardware_interface/visibility_control.h"
#include "rmd_hardware_interface/byte_swap.hpp"
#include "rmd_hardware_interface/socket_can.hpp"
#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#define DEG2RAD (2 * M_PI)/360.0
#define RAD2DEG 360.0/(2 * M_PI)
#define ROT2RAD (2 * M_PI)
#define RAD2ROT 1/(2 * M_PI)
#define SINGLEMOTOR 0x140
#define MULTIMOTOR 0x280

namespace rmd_hardware_interface
{
  enum Params : uint32_t 
  {
    torque_constant,
    max_velocity,
    gear_ratio,
    range,
  };
  enum ControlModes : uint32_t 
  {
    TORQUE,
    VELOCITY,
    POSITION
  };
  enum CommandIDs : uint32_t 
  {
    read_pos_kp = 0x30,
    read_pos_ki = 0x31,
    read_vel_kp = 0x32,
    read_vel_ki = 0x33,
    read_torque_kp = 0x34,
    read_torque_ki = 0x35,
    write_pos_kp_ram = 0x36,
    write_pos_ki_ram = 0x37,
    write_vel_kp_ram = 0x38,
    write_vel_ki_ram = 0x39,
    write_torque_kp_ram = 0x3A,
    write_torque_ki_ram = 0x3B,
    write_pos_kp_rom = 0x3C,
    write_pos_ki_rom = 0x3D,
    write_vel_kp_rom = 0x3E,
    write_vel_ki_rom = 0x3F,
    write_torque_kp_rom = 0x40,
    write_torque_ki_rom = 0x41,
    read_acc = 0x42,
    write_acc_ram = 0x43,
    read_multiturn_counts = 0x60,
    read_multiturn_counts_raw = 0x61,
    read_multiturn_counts_offset = 0x62,
    write_custom_pos_aszero_rom = 0x63,
    write_current_pos_aszero_rom = 0x64,
    read_multiturn_angles = 0x92,
    read_motor_status_1 = 0x9A,
    read_motor_status_2 = 0x9C,
    read_motor_status_3 = 0x9D,
    motor_off = 0x80,
    motor_stop = 0x81,
    motor_run = 0x88,
    torque_closed_loop = 0xA1,
    velocity_closed_loop = 0xA2,
    position_closed_loop_1 = 0xA3,
    position_closed_loop_2 = 0xA4,
    position_closed_loop_3 = 0xA5,
    position_closed_loop_4 = 0xA6,
    multiturn_incremental_pos = 0xA7,
    read_running_mode = 0x70,
    read_power_status = 0x71,
    read_battery_voltage = 0x72,
    write_feedforward  = 0x73,
    system_reset = 0x76,
    open_brake = 0x77,
    close_brake = 0x78,
    readwrite_can_id = 0x79,
  };
class RMDHardwareInterface : public hardware_interface::ActuatorInterface
{
public:
  RMD_HARDWARE_INTERFACE_PUBLIC
  virtual ~RMDHardwareInterface();

  RMD_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  RMD_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  RMD_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  RMD_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  RMD_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  RMD_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  RMD_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  RMD_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Function to receive CAN messages
  void recv_callback(const can_frame & frame);

private:
  // CAN Parameters
  std::string can_interface_;
  EpollEventLoop event_loop_;
  std::unique_ptr<std::thread> can_event_loop;
  SocketCanIntf can_intf_ = SocketCanIntf();
  bool activated;

  std::unordered_map<std::string, ControlModes> control_modes = {
  {"position",ControlModes::POSITION},
  {"velocity",ControlModes::VELOCITY},
  {"torque",ControlModes::TORQUE}};

  std::unordered_map<int, std::string> error_codes_ = {
  {0x0000, "Hardware Over Current"},
  {0x0002, "Motor Stalled"},
  {0x0004, "Low Voltage"},
  {0x0008, "Over Voltage"},
  {0x0010, "Over Current"},
  {0x0020, "Brake Opening Failed"},
  {0x0040, "Bus Current Error"},
  {0x0080, "Battery Voltage Error"},
  {0x0100, "Overspeed"},
  {0x0200, "Position Loop Exceeded Error"},
  {0x0400, "VDD Error"},
  {0x0800, "DSP Internal Sensor Temperature is Overheated"},
  {0x1000, "Motor Temperature is Overheated"},
  {0x2000, "Encoder Calibration Error"},
  {0x00F0, "PID parameter write ROM protection, non-safe operation"},
  {0x00F1, "Encoder value is written into ROM protection, non-safe operation"},
  {0x00F2, "Three-loop switching operation error, non-safe operation"},
  {0x00F3, "Motor brake is not open"},
  {0x00F4, "Motor write ROM protection, non-safe operation"},};
  
  std::unordered_map<std::string, std::vector<double>> supported_motors_ = {
  {"RMDX8_19",  {2.09,22.5,9,0.6981317}},
  {"RMDX6_16",  {0.88,19.9,6,1.04719755}}};
  
  struct Motor {
    uint32_t node_id;
    uint16_t error_code;
    std::string model;
    ControlModes control_mode;
    double pos_kp;
    double pos_ki;
    double speed_kp;
    double speed_ki;
    double current_kp;
    double current_ki;
    double reduction;
    double offset;
    double wrap_offset;
    double prev_wrap_position_rad;
    double curr_wrap_position_rad;
    double raw_position_rad;
    double raw_velocity_rad_s;
    double raw_torque_n_m;
    double raw_acceleration_rad_ss;
    double hw_commands_positions_rad;
    double hw_commands_velocities_rad_s;
    double hw_commands_efforts_n_m;
    double hw_states_positions_rad;
    double hw_states_velocities_rad_s;
    double hw_states_efforts_n_m;
    double homing_offset;
    double homing_vel;
    double torque_constant;
    double max_velocity;
    double gear_ratio;
    double range;
    int8_t temperature;
    bool homing_done;
    bool home_on_startup;
    bool endstop_state;
    bool endstop_detected;
    bool response_received;
  };

  std::vector<Motor> motor_;

  double map(double value, double fromLow, double fromHigh, double toLow, double toHigh) {
    return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
  }

  void continuous_pos(Motor* motor, double discontinuous_pos)
  {
    // Logic to convert discontinuous counts into continuous counts by detecting wraparound
    motor->prev_wrap_position_rad = motor->curr_wrap_position_rad;
    motor->curr_wrap_position_rad = discontinuous_pos;
    double delta =  motor->curr_wrap_position_rad - motor->prev_wrap_position_rad;
    if (std::abs(delta)>(motor->range/2))
    {
      if (delta > 0) {
          motor->wrap_offset -= (motor->range);
      } else {
          motor->wrap_offset += (motor->range);
      }
    }
    motor->raw_position_rad = motor->wrap_offset + motor->curr_wrap_position_rad - motor->homing_offset;
  }

  void request(Motor* motor, CommandIDs command_id)
  {
    struct can_frame frame;
    frame.can_id = SINGLEMOTOR|motor->node_id;
    frame.len = 8;
    memset(frame.data, 0, sizeof(frame.data));
    frame.data[0] = static_cast<uint8_t>(command_id);
    can_intf_.send_can_frame(frame); 
  }

  void send_torque(Motor* motor, double torque_nm)
  {
    struct can_frame frame;
    frame.can_id = SINGLEMOTOR|motor->node_id;
    frame.len = 8;
    memset(frame.data, 0, sizeof(frame.data));
    frame.data[0] = CommandIDs::torque_closed_loop;
    write_le<int16_t>((int)map(torque_nm/motor->torque_constant,-32,32,-2000,2000), frame.data + 4);
    can_intf_.send_can_frame(frame); 
  }

  void send_vel(Motor* motor, double speed_rad_s)
  {
    struct can_frame frame;
    frame.can_id = SINGLEMOTOR|motor->node_id;
    frame.len = 8;
    memset(frame.data, 0, sizeof(frame.data));
    frame.data[0] = CommandIDs::velocity_closed_loop;
    write_le<int32_t>((int32_t)(speed_rad_s*RAD2DEG*100.0*motor->gear_ratio), frame.data + 4);
    can_intf_.send_can_frame(frame); 
  }

};

}  // namespace rmd_hardware_interface

#endif  // RMD_HARDWARE_INTERFACE__RMD_HARDWARE_INTERFACE_HPP_
