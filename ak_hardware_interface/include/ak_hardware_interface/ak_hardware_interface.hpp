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

#ifndef AK_HARDWARE_INTERFACE__AK_HARDWARE_INTERFACE_HPP_
#define AK_HARDWARE_INTERFACE__AK_HARDWARE_INTERFACE_HPP_

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

#include "ak_hardware_interface/visibility_control.h"
#include "ak_hardware_interface/byte_swap.hpp"
#include "ak_hardware_interface/socket_can.hpp"
#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace ak_hardware_interface
{
  enum Params : uint32_t 
  {
    P_min,
    P_max,
    V_min,
    V_max,
    T_min,
    T_max,
    Kp_min,
    Kp_max,
    Kd_min,
    Kd_max,
    Kt_TMotor,
    Current_Factor,
    Kt_actual,
    GEAR_RATIO
  };
  enum ControlModes : uint32_t 
  {
    TORQUE,
    VELOCITY,
    POSITION
  };
class AKHardwareInterface : public hardware_interface::ActuatorInterface
{
public:
  AK_HARDWARE_INTERFACE_PUBLIC
  virtual ~AKHardwareInterface();

  AK_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  AK_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  AK_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  AK_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  AK_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  AK_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  AK_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  AK_HARDWARE_INTERFACE_PUBLIC
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

  std::unordered_map<std::string, ControlModes> control_modes = {
  {"position",ControlModes::POSITION},
  {"velocity",ControlModes::VELOCITY},
  {"torque",ControlModes::TORQUE}};

  std::unordered_map<int, std::string> error_codes_ = {
  {0, "No Error"},
  {1, "Over temperature fault"},
  {2, "Over current fault"},
  {3, "Over voltage fault"},
  {4, "Under voltage fault"},
  {5, "Encoder fault"},
  {6, "Phase current unbalance fault (The hardware may be damaged)"}};
  
  std::unordered_map<std::string, std::vector<double>> supported_motors_ = {
  {"AK80_9",  {-12.5,12.5,-50.0,50.0,-18.0,18.0,0.0,500.0,0.0,5.0,0.091,0.59,0.115,9.0}},
  {"AK10_9",  {-12.5,12.5,-50.0,50.0,-65.0,65.0,0.0,500.0,0.0,5.0,0.160,0.59,0.206,9.0}},
  {"AK60_6",  {-12.5,12.5,-50.0,50.0,-15.0,15.0,0.0,500.0,0.0,5.0,0.068,0.59,0.087,6.0}},
  {"AK70_10", {-12.5,12.5,-50.0,50.0,-25.0,25.0,0.0,500.0,0.0,5.0,0.095,0.59,0.122,10.0}},
  {"AK80_6",  {-12.5,12.5,-76.0,76.0,-12.0,12.0,0.0,500.0,0.0,5.0,0.091,0.59,0.017,6.0}},
  {"AK80_64", {-12.5,12.5,-8.0,8.0,-144.0,144.0,0.0,500.0,0.0,5.0,0.119,0.59,0.153,80.0}}};
  
  struct Motor {
    uint32_t node_id;
    std::string model;
    uint8_t current_temp;
    uint8_t error_code;
    uint8_t control_mode;
    double kp;
    double kd;
    double P_min;
    double P_max;
    double V_min;
    double V_max;
    double T_min;
    double T_max;
    double Kp_min;
    double Kp_max;
    double Kd_min;
    double Kd_max;
    double Kt_TMotor;
    double Current_Factor;
    double Kt_actual;
    double GEAR_RATIO;
    double Range;
    double reduction;
    double offset;
    double wrap_offset;
    double prev_wrap_position_rad;
    double curr_wrap_position_rad;
    double raw_position_rad;
    double raw_velocity_rad_s;
    double raw_torque_n_m;
    double hw_commands_positions_rad;
    double hw_commands_velocities_rad_s;
    double hw_commands_efforts_n_m;
    double hw_states_positions_rad;
    double hw_states_velocities_rad_s;
    double hw_states_efforts_n_m;
  };

  std::vector<Motor> motor_;

  int float_to_uint(float x, float x_min, float x_max, unsigned int bits)
  {
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float bitratio = (1 << bits) / span;
    
    x = std::clamp(x, x_min, x_max - (2 / bitratio));
    
    return static_cast<uint32_t>(std::clamp((x - x_min) * bitratio, (float)0.0, (x_max - x_min) * bitratio));
  }

  float uint_to_float(int x_int, float x_min, float x_max, int bits)
  {
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
  }
};

}  // namespace ak_hardware_interface

#endif  // AK_HARDWARE_INTERFACE__AK_HARDWARE_INTERFACE_HPP_
