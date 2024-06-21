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
  auto it = info_.hardware_parameters.find("interface");
  if (it == info_.hardware_parameters.end()) {
    RCLCPP_FATAL(
      rclcpp::get_logger(
        "AKHardwareInterface"),
      "Parameter 'interface' not set. It is required for Socket CAN.");
    return hardware_interface::CallbackReturn::ERROR;
  } else {
    can_interface_ = it->second;
  }

  motor_.resize(info_.joints.size(), std::numeric_limits<Motor>::quiet_NaN());

  for (uint i = 0; i < info_.joints.size(); i++)
  {
    it = info_.joints[i].parameters.find("node_id");
    if (it == info_.joints[i].parameters.end())
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(
          "AKHardwareInterface"),
        "Parameter 'node_id' not set for '%s'. It is required for Socket CAN.",
        info_.joints[i].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    else
    {
      motor_[i].node_id = std::stoi(it->second);
    }
    it = info_.joints[i].parameters.find("model");
    if (it == info_.joints[i].parameters.end())
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(
          "AKHardwareInterface"),
        "Parameter 'model' not set for '%s'. Valid values are AK80_9, AK10_9, AK60_6, AK70_10, AK80_6, AK80_64.",
        info_.joints[i].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    else
    {
      motor_[i].model = it->second;
    }
    it = info_.joints[i].parameters.find("control_mode");
    if (it == info_.joints[i].parameters.end())
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(
          "AKHardwareInterface"),
        "Parameter 'control_mode' not set for '%s'. Valid values are torque, velocity, position.",
        info_.joints[i].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    else
    {
      motor_[i].control_mode = control_modes.at(it->second);
    }
    it = info_.joints[i].parameters.find("reduction");
    if (it == info_.joints[i].parameters.end())
    {
      RCLCPP_INFO(
        rclcpp::get_logger(
          "AKHardwareInterface"),
        "Parameter 'reduction' not set for '%s'. Defaulting to 1.0.",
        info_.joints[i].name.c_str());
      motor_[i].reduction = 1.0;
    }
    else
    {
      motor_[i].reduction = std::stod(it->second);
      RCLCPP_INFO(
        rclcpp::get_logger(
          "AKHardwareInterface"),
        "Parameter 'reduction' set for '%s' to %lf.",
        info_.joints[i].name.c_str(),motor_[i].reduction);
    }
    it = info_.joints[i].parameters.find("offset");
    if (it == info_.joints[i].parameters.end())
    {
      RCLCPP_INFO(
        rclcpp::get_logger(
          "AKHardwareInterface"),
        "Parameter 'offset' not set for '%s'. Defaulting to 0.0.",
        info_.joints[i].name.c_str());
      motor_[i].offset = 0.0;
    }
    else
    {
      motor_[i].offset = std::stod(it->second);
      RCLCPP_INFO(
        rclcpp::get_logger(
          "AKHardwareInterface"),
        "Parameter 'offset' set for '%s' to %lf.",
        info_.joints[i].name.c_str(),motor_[i].offset);
    }
    it = info_.joints[i].parameters.find("kp");
    if (it == info_.joints[i].parameters.end())
    {
      RCLCPP_INFO(
        rclcpp::get_logger(
          "AKHardwareInterface"),
        "Parameter 'kp' not set for '%s'. Defaulting to 0.0.",
        info_.joints[i].name.c_str());
      motor_[i].kp = 0.0;
    }
    else
    {
      motor_[i].kp = std::stod(it->second);
      RCLCPP_INFO(
        rclcpp::get_logger(
          "AKHardwareInterface"),
        "Parameter 'kp' set for '%s' to %lf.",
        info_.joints[i].name.c_str(),motor_[i].kp);
    }
    it = info_.joints[i].parameters.find("kd");
    if (it == info_.joints[i].parameters.end())
    {
      RCLCPP_INFO(
        rclcpp::get_logger(
          "AKHardwareInterface"),
        "Parameter 'kd' not set for '%s'. Defaulting to 0.0.",
        info_.joints[i].name.c_str());
      motor_[i].kd = 0.0;
    }
    else
    {
      motor_[i].kd = std::stod(it->second);
      RCLCPP_INFO(
        rclcpp::get_logger(
          "AKHardwareInterface"),
        "Parameter 'kd' set for '%s' to %lf.",
        info_.joints[i].name.c_str(),motor_[i].kd);
    }
    it = info_.joints[i].parameters.find("home_on_startup");
    if (it == info_.joints[i].parameters.end()) {
      RCLCPP_INFO(
        rclcpp::get_logger(
          "AKHardwareInterface"),
        "Parameter 'home_on_startup' not set for '%s'. Defaulting to False.",
        info_.joints[i].name.c_str());
      motor_[i].home_on_startup = false;
    }
    else
    {
      const std::string home_on_startup = it->second;
      if (home_on_startup == "True") {
        motor_[i].home_on_startup = true;
      } else if (home_on_startup == "False") {
        motor_[i].home_on_startup = false;
      } else {
        RCLCPP_WARN(
          rclcpp::get_logger(
            "AKHardwareInterface"),
          "Invalid home_on_startup param value '%s'. Allowed values are True and False. Defaulting to False.",
          home_on_startup.c_str());
        motor_[i].home_on_startup = false;
      RCLCPP_INFO(
        rclcpp::get_logger(
          "AKHardwareInterface"),
        "Parameter 'home_on_startup' set for '%s' to %d.",
        info_.joints[i].name.c_str(),motor_[i].home_on_startup);
        }
    }
    if(motor_[i].home_on_startup)
    {
      it = info_.joints[i].parameters.find("homing_torque");
      if (it == info_.joints[i].parameters.end())
      {
        RCLCPP_FATAL(
          rclcpp::get_logger(
            "AKHardwareInterface"),
          "Parameter 'homing_torque' not set for '%s' despite 'home_on_startup' being set true. It is required for homing procedure.", info_.joints[i].name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      else
      {
        motor_[i].homing_torque = std::stod(it->second);
        RCLCPP_INFO(
          rclcpp::get_logger(
            "AKHardwareInterface"),
          "Parameter 'homing_torque' set for '%s' to %lf.",
          info_.joints[i].name.c_str(),motor_[i].homing_torque);
      }
    }
    else
    {
      motor_[i].homing_torque = 0.0;
    }
  }

  // Initialise Motor Params
  for (uint i = 0; i < info_.joints.size(); i++) {
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
    motor_[i].Range = motor_[i].P_max- motor_[i].P_min;
  }

  activated = false;

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AKHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // reset values always when configuring hardware
  for (uint i = 0; i < info_.joints.size(); i++) {
    motor_[i].hw_commands_positions_rad = 0;
    motor_[i].hw_commands_velocities_rad_s = 0;
    motor_[i].hw_commands_efforts_n_m = 0;
    motor_[i].hw_states_positions_rad = 0;
    motor_[i].hw_states_velocities_rad_s = 0;
    motor_[i].hw_states_efforts_n_m = 0;
    motor_[i].prev_wrap_position_rad = 0;
    motor_[i].curr_wrap_position_rad = 0;
    motor_[i].raw_position_rad = 0;
    motor_[i].wrap_offset = 0;
    motor_[i].homing_offset = 0;
    motor_[i].endstop_state = false;
    motor_[i].endstop_detected = false;
    motor_[i].homing_done = false;
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
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &motor_[i].hw_states_positions_rad));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &motor_[i].hw_states_velocities_rad_s));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &motor_[i].hw_states_efforts_n_m));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> AKHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &motor_[i].hw_commands_positions_rad));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &motor_[i].hw_commands_velocities_rad_s));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &motor_[i].hw_commands_efforts_n_m));
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

  for (uint i = 0; i < info_.joints.size(); i++)
  {
    RCLCPP_INFO(
        rclcpp::get_logger("AKHardwareInterface"), "'%s': Sending Activation",info_.joints[i].name.c_str());
    while(!activate_motor(&motor_[i]));
      RCLCPP_INFO(
        rclcpp::get_logger("AKHardwareInterface"), "'%s': Got Reply",info_.joints[i].name.c_str());

    if(motor_[i].home_on_startup)
    {
      RCLCPP_INFO(
        rclcpp::get_logger("AKHardwareInterface"), "'%s': Sending Homing Torque",info_.joints[i].name.c_str());
      while(!send_torque(&motor_[i],motor_[i].homing_torque));
      RCLCPP_INFO(
        rclcpp::get_logger("AKHardwareInterface"), "'%s': Got Reply",info_.joints[i].name.c_str());
      RCLCPP_INFO(
        rclcpp::get_logger("AKHardwareInterface"), "'%s': Waiting for Endstop",info_.joints[i].name.c_str());
      while(!motor_[i].endstop_state);
      RCLCPP_INFO(
        rclcpp::get_logger("AKHardwareInterface"), "'%s': Got Endstop",info_.joints[i].name.c_str());
      motor_[i].homing_offset = motor_[i].raw_position_rad;
      while(!send_torque(&motor_[i],0.0));
      RCLCPP_INFO(
        rclcpp::get_logger("AKHardwareInterface"), "Homing Offset for '%s' detected as %lf",info_.joints[i].name.c_str(),motor_[i].homing_offset);
      while(motor_[i].raw_velocity_rad_s>1e-2);
      motor_[i].homing_done = true;
    }
  }
  activated = true;
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
    if ((frame.len!=6)&&(frame.len!=8)&&(frame.len!=2))
    {
      RCLCPP_WARN(rclcpp::get_logger("AKHardwareInterface"),
            "Incorrect frame length received from motor with ID: %d. %d != 2 or 6 or 8", frame.can_id, frame.len);
      continue;
    }
    motor_[i].response_received = true;
    if(frame.len == 8)
    {
      motor_[i].current_temp = int(frame.data[6]);
      motor_[i].error_code = int(frame.data[7]);
    }
    if((frame.len == 2))
    {
      motor_[i].endstop_state = bool(frame.data[1]);
      RCLCPP_INFO(
        rclcpp::get_logger("AKHardwareInterface"), "'%s': Got Endstop as %d",info_.joints[i].name.c_str(),motor_[i].endstop_state);
      continue;
    }

    uint32_t position_uint = frame.data[1] <<8 | frame.data[2];
    uint32_t velocity_uint = ((frame.data[3] << 8) | (frame.data[4]>>4) <<4 ) >> 4;
    uint32_t torque_uint = (frame.data[4]&0x0F)<<8 | frame.data[5];

    motor_[i].raw_velocity_rad_s = uint_to_float(velocity_uint,motor_[i].V_min,motor_[i].V_max,12);
    motor_[i].raw_torque_n_m = uint_to_float(torque_uint,motor_[i].T_min,motor_[i].T_max,12);

    // Logic to convert discontinuous counts into continuous counts by detecting wraparound
    motor_[i].prev_wrap_position_rad = motor_[i].curr_wrap_position_rad;
    motor_[i].curr_wrap_position_rad = uint_to_float(position_uint,motor_[i].P_min,motor_[i].P_max,16);
    double delta =  motor_[i].curr_wrap_position_rad - motor_[i].prev_wrap_position_rad;
    if (std::abs(delta)>(motor_[i].Range/2))
    {
      if (delta > 0) {
          motor_[i].wrap_offset -= (motor_[i].Range);
      } else {
          motor_[i].wrap_offset += (motor_[i].Range);
      }
    }
    motor_[i].raw_position_rad = motor_[i].wrap_offset + motor_[i].curr_wrap_position_rad - motor_[i].homing_offset;
  }
}

hardware_interface::CallbackReturn AKHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("AKHardwareInterface"), "Stopping ...please wait...");
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    deactivate_motor(&motor_[i]);
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
    if(motor_[i].error_code!=0)
    {
      std::string error_message = error_codes_[motor_[i].error_code];
      RCLCPP_FATAL(
        rclcpp::get_logger(
          "AKHardwareInterface"),
        "Error code received for '%s': %s",
        info_.joints[i].name.c_str(),error_message.c_str());
      return hardware_interface::return_type::ERROR;
    }
    motor_[i].hw_states_efforts_n_m = motor_[i].raw_torque_n_m * motor_[i].reduction;
    motor_[i].hw_states_positions_rad = (motor_[i].raw_position_rad / motor_[i].reduction) + motor_[i].offset;
    motor_[i].hw_states_velocities_rad_s = motor_[i].raw_velocity_rad_s / motor_[i].reduction;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AKHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if(!activated)
  {
    return hardware_interface::return_type::OK;
  }

  for (uint i = 0; i < info_.joints.size(); i++)
  {
    double position = ((motor_[i].hw_commands_positions_rad - motor_[i].offset) * motor_[i].reduction) - motor_[i].homing_offset;
    double velocity = motor_[i].hw_commands_velocities_rad_s * motor_[i].reduction;
    double torque = motor_[i].hw_commands_efforts_n_m / motor_[i].reduction;
    double kp = motor_[i].kp;
    double kd = motor_[i].kd;
    switch(motor_[i].control_mode)
    {
      case ControlModes::TORQUE:
      {
        send_torque(&motor_[i],torque);
      }
      break;
      case ControlModes::VELOCITY:
      {
        send_velocity(&motor_[i],velocity,kd);
      }
      break;
      case ControlModes::POSITION:
      {
        send_position(&motor_[i],position,kp);
      }
      break;
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace ak_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ak_hardware_interface::AKHardwareInterface, hardware_interface::ActuatorInterface)
