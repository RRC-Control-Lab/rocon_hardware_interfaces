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

#include <limits>
#include <vector>

#include "ak_hardware_interface/ak_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ak_hardware_interface
{
AKHardwareInterface::~AKHardwareInterface()
{
  // If the controller manager is shutdown via Ctrl + C the on_deactivate methods won't be called.
  // We therefore need to make sure to actually deactivate the communication
  on_deactivate(rclcpp_lifecycle::State());
}
hardware_interface::CallbackReturn AKHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::ActuatorInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // Read the parameters
  can_interface_ = info_.hardware_parameters["interface"];

  // Set all vectors to appropriate size and initialise to NaN
  hw_states_positions_rad_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocities_rad_s_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_efforts_n_m_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_positions_rad_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_rad_s_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_efforts_n_m_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  
  motor_.resize(info_.joints.size(), std::numeric_limits<Motor>::quiet_NaN());

  // Initialise Motor Params
  for (uint i = 0; i < info_.joints.size(); i++) {
    motor_[i].node_id = std::stoi(info_.joints[i].parameters.at("node_id"));
    motor_[i].model = info_.joints[i].parameters.at("model");
    motor_[i].control_mode = control_modes.at(info_.joints[i].parameters.at("control_mode"));
    motor_[i].P_min = supported_motors_.at(motor_[i].model)[Params::P_min];
    motor_[i].P_max = supported_motors_.at(motor_[i].model)[Params::P_max];
    motor_[i].V_min = supported_motors_.at(motor_[i].model)[Params::V_min];
    motor_[i].V_max = supported_motors_.at(motor_[i].model)[Params::V_max];
    motor_[i].T_min = supported_motors_.at(motor_[i].model)[Params::T_min];
    motor_[i].T_max = supported_motors_.at(motor_[i].model)[Params::T_max];
    motor_[i].Kp_min = supported_motors_.at(motor_[i].model)[Params::Kp_min];
    motor_[i].Kp_max = supported_motors_.at(motor_[i].model)[Params::Kp_max];
    motor_[i].Kd_min = supported_motors_.at(motor_[i].model)[Params::Kd_min];
    motor_[i].Kd_max = supported_motors_.at(motor_[i].model)[Params::Kd_max];
    motor_[i].Kt_TMotor = supported_motors_.at(motor_[i].model)[Params::Kt_TMotor];
    motor_[i].Current_Factor = supported_motors_.at(motor_[i].model)[Params::Current_Factor];
    motor_[i].Kt_actual = supported_motors_.at(motor_[i].model)[Params::Kt_actual];
    motor_[i].GEAR_RATIO = supported_motors_.at(motor_[i].model)[Params::GEAR_RATIO];
  }

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AKHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to be ready for read calls and write calls of some interfaces

  // reset values always when configuring hardware
  for (uint i = 0; i < info_.joints.size(); i++) {
    hw_commands_positions_rad_[i] = 0;
    hw_commands_velocities_rad_s_[i] = 0;
    hw_commands_efforts_n_m_[i] = 0;
    hw_states_positions_rad_[i] = 0;
    hw_states_velocities_rad_s_[i] = 0;
    hw_states_efforts_n_m_[i] = 0;
  }
  RCLCPP_INFO(rclcpp::get_logger("AKHardwareInterface"), "Successfully configured!");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> AKHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_rad_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_rad_s_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_efforts_n_m_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> AKHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_rad_[i]));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_rad_s_[i]));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_efforts_n_m_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn AKHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("AKHardwareInterface"), "Activating AKHardwareInterface");

  if (!can_intf_.init(
      can_interface_, &event_loop_,
      std::bind(&AKHardwareInterface::recv_callback, this, _1)))
  {
    RCLCPP_INFO(
      rclcpp::get_logger(
        "AKHardwareInterface"), "Failed to initialize socket can interface: %s",
      can_interface_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  can_event_loop = std::make_unique<std::thread>([&]() {event_loop_.run_until_empty();});

  struct can_frame frame;
  frame.len = 8;
  const uint8_t data_values[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
  std::copy(data_values, data_values + 8, frame.data);
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    frame.can_id = motor_[i].node_id;
    can_intf_.send_can_frame(frame);
  }
  std::this_thread::sleep_for(std::chrono::seconds(3));
  return CallbackReturn::SUCCESS;
}

void AKHardwareInterface::recv_callback(const can_frame & frame)
{
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    if ((frame.data[0]) != motor_[i].node_id)
    {
      continue;
    }
    if ((frame.len!=6)&&(frame.len!=8))
    {
      RCLCPP_WARN(rclcpp::get_logger("AKHardwareInterface"),
            "Incorrect frame length received from motor with ID: %d. %d != 6 or 8", frame.can_id, frame.len);
      continue;
    }
    if(frame.len == 8)
    {
      motor_[i].current_temp = int(frame.data[6]);
      motor_[i].error_code = int(frame.data[7]);
    }

    uint32_t position_uint = frame.data[1] <<8 | frame.data[2];
    uint32_t velocity_uint = ((frame.data[3] << 8) | (frame.data[4]>>4) <<4 ) >> 4;
    uint32_t torque_uint = (frame.data[4]&0x0F)<<8 | frame.data[5];

    motor_[i].current_position = uint_to_float(position_uint,motor_[i].P_min,motor_[i].P_max,16);
    motor_[i].current_velocity = uint_to_float(velocity_uint,motor_[i].V_min,motor_[i].V_max,12);
    motor_[i].current_torque = uint_to_float(torque_uint,motor_[i].T_min,motor_[i].T_max,12);
  }
}

hardware_interface::CallbackReturn AKHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("AKHardwareInterface"), "Stopping ...please wait...");

  struct can_frame frame;
  frame.len = 8;
  const uint8_t data_values[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
  std::copy(data_values, data_values + 8, frame.data);
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    frame.can_id = motor_[i].node_id;
    can_intf_.send_can_frame(frame);
  }

  can_intf_.deinit();

  // Ensure the thread is joined and resources are cleaned up
  if (can_event_loop->joinable()) {
    can_event_loop->join();
  }

  RCLCPP_INFO(rclcpp::get_logger("AKHardwareInterface"), "System successfully stopped!");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type AKHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    hw_states_efforts_n_m_[i] = motor_[i].current_torque;
    hw_states_velocities_rad_s_[i] = motor_[i].current_velocity;
    hw_states_positions_rad_[i] = motor_[i].current_position;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AKHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  struct can_frame frame;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    frame = can_frame{};
    frame.can_id = motor_[i].node_id;
    frame.len = 8;
    float p_des, v_des, kp, kd, t_ff = 0;
    switch(motor_[i].control_mode)
    {
      case ControlModes::TORQUE:
      {
        p_des = 0;
        v_des = 0;
        kp = 0;
        kd = 0;
        // int8_t torque_sign = (hw_commands_efforts_n_m_[i] > 0) - (hw_commands_efforts_n_m_[i] < 0);
        // int8_t is_stopped = fabs(hw_states_velocities_rad_s_[i])<0.05;
        // double torque_mag = fabs(hw_commands_efforts_n_m_[i]) + 0.7 + (is_stopped*1.3);
        t_ff = fminf(fmaxf(motor_[i].T_min, hw_commands_efforts_n_m_[i]), motor_[i].T_max);
      }
      break;
      case ControlModes::VELOCITY:
      {
        p_des = 0;
        v_des = fminf(fmaxf(motor_[i].V_min, hw_commands_velocities_rad_s_[i]), motor_[i].V_max);
        kp = 0;
        kd = fminf(fmaxf(motor_[i].Kd_min, kd), motor_[i].Kd_max);
        t_ff = 0;
      }
      break;
      case ControlModes::POSITION:
      {
        p_des = fminf(fmaxf(motor_[i].P_min, hw_commands_positions_rad_[i]), motor_[i].P_max);
        v_des = 0;
        kp = fminf(fmaxf(motor_[i].Kp_min, kp), motor_[i].Kp_max);
        kd = 0;
        t_ff = 0;
      }
      break;
    }
    /// convert floats to unsigned ints ///
    uint16_t p_int = float_to_uint(p_des, motor_[i].P_min, motor_[i].P_max, 16);
    uint16_t v_int = float_to_uint(v_des, motor_[i].V_min, motor_[i].V_max, 12);
    uint16_t kp_int = float_to_uint(kp, motor_[i].Kp_min, motor_[i].Kp_max, 12);
    uint16_t kd_int = float_to_uint(kd, motor_[i].Kd_min, motor_[i].Kd_max, 12);
    uint16_t t_int = float_to_uint(t_ff, motor_[i].T_min, motor_[i].T_max, 12);
    /// pack ints into the can buffer ///
    frame.data[0] = p_int>>8; // Position 8 higher
    frame.data[1] = p_int&0xFF; // Position 8 lower
    frame.data[2] = v_int>>4; // Speed 8 higher
    frame.data[3] = ((v_int&0xF)<<4)|(kp_int>>8); //Speed 4 bit lower KP 4bit higher
    frame.data[4] = kp_int&0xFF; // KP 8 bit lower
    frame.data[5] = kd_int>>4; // Kd 8 bit higher
    frame.data[6] = ((kd_int&0xF)<<4)|(t_int>>8); // KP 4 bit lower torque 4 bit higher
    frame.data[7] = t_int&0xff; // torque 4 bit lower

    can_intf_.send_can_frame(frame);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace ak_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ak_hardware_interface::AKHardwareInterface, hardware_interface::ActuatorInterface)
