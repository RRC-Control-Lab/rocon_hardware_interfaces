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

#include "rmd_hardware_interface/rmd_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rmd_hardware_interface
{
RMDHardwareInterface::~RMDHardwareInterface()
{
  // If the controller manager is shutdown via Ctrl + C the on_deactivate methods won't be called.
  // We therefore need to make sure to actually deactivate the communication
  on_deactivate(rclcpp_lifecycle::State());
}
hardware_interface::CallbackReturn RMDHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::ActuatorInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // Read the parameters
  if (info_.hardware_parameters.find("interface") == info_.hardware_parameters.end()) {
    RCLCPP_FATAL(
      rclcpp::get_logger(
        "RMDHardwareInterface"),
      "Parameter 'interface' not set. It is required for Socket CAN.");
    return hardware_interface::CallbackReturn::ERROR;
  } else {
    can_interface_ = info_.hardware_parameters["interface"];
  }

  motor_.resize(info_.joints.size(), std::numeric_limits<Motor>::quiet_NaN());

  for (uint i = 0; i < info_.joints.size(); i++)
  {
    if (info_.joints[i].parameters.find("node_id") == info_.joints[i].parameters.end())
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(
          "RMDHardwareInterface"),
        "Parameter 'node_id' not set for '%s'. It is required for Socket CAN.",
        info_.joints[i].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    else
    {
      motor_[i].node_id = std::stoi(info_.joints[i].parameters["node_id"]);
    }
    if (info_.joints[i].parameters.find("model") == info_.joints[i].parameters.end())
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(
          "RMDHardwareInterface"),
        "Parameter 'model' not set for '%s'. Valid values are 'RMDX8_19'.",
        info_.joints[i].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    else
    {
      motor_[i].model = info_.joints[i].parameters["model"];
    }
    if (info_.joints[i].parameters.find("control_mode") == info_.joints[i].parameters.end())
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(
          "RMDHardwareInterface"),
        "Parameter 'control_mode' not set for '%s'. Valid values are torque, velocity, position.",
        info_.joints[i].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    else
    {
      motor_[i].control_mode = control_modes.at(info_.joints[i].parameters["control_mode"]);
    }
    if (info_.joints[i].parameters.find("reduction") == info_.joints[i].parameters.end())
    {
      RCLCPP_INFO(
        rclcpp::get_logger(
          "RMDHardwareInterface"),
        "Parameter 'reduction' not set for '%s'. Defaulting to 1.0.",
        info_.joints[i].name.c_str());
      motor_[i].reduction = 1.0;
    }
    else
    {
      motor_[i].reduction = std::stod(info_.joints[i].parameters["reduction"]);
      RCLCPP_INFO(
        rclcpp::get_logger(
          "RMDHardwareInterface"),
        "Parameter 'reduction' set for '%s' to %lf.",
        info_.joints[i].name.c_str(),motor_[i].reduction);
    }
    if (info_.joints[i].parameters.find("offset") == info_.joints[i].parameters.end())
    {
      RCLCPP_INFO(
        rclcpp::get_logger(
          "RMDHardwareInterface"),
        "Parameter 'offset' not set for '%s'. Defaulting to 0.0.",
        info_.joints[i].name.c_str());
      motor_[i].offset = 0.0;
    }
    else
    {
      motor_[i].offset = std::stod(info_.joints[i].parameters["offset"]);
      RCLCPP_INFO(
        rclcpp::get_logger(
          "RMDHardwareInterface"),
        "Parameter 'offset' set for '%s' to %lf.",
        info_.joints[i].name.c_str(),motor_[i].offset);
    }
    if (info_.joints[i].parameters.find("home_on_startup") == info_.joints[i].parameters.end()) {
      RCLCPP_INFO(
        rclcpp::get_logger(
          "RMDHardwareInterface"),
        "Parameter 'home_on_startup' not set for '%s'. Defaulting to False.",
        info_.joints[i].name.c_str());
      motor_[i].home_on_startup = false;
    }
    else
    {
      const std::string home_on_startup = info_.joints[i].parameters["home_on_startup"];
      if (home_on_startup == "True") {
        motor_[i].home_on_startup = true;
      } else if (home_on_startup == "False") {
        motor_[i].home_on_startup = false;
      } else {
        RCLCPP_WARN(
          rclcpp::get_logger(
            "RMDHardwareInterface"),
          "Invalid home_on_startup param value '%s'. Allowed values are True and False. Defaulting to False.",
          home_on_startup.c_str());
        motor_[i].home_on_startup = false;
      RCLCPP_INFO(
        rclcpp::get_logger(
          "RMDHardwareInterface"),
        "Parameter 'home_on_startup' set for '%s' to %d.",
        info_.joints[i].name.c_str(),motor_[i].home_on_startup);
        }
    }
    if(motor_[i].home_on_startup)
    {
      if (info_.joints[i].parameters.find("homing_vel") == info_.joints[i].parameters.end())
      {
        RCLCPP_FATAL(
          rclcpp::get_logger(
            "RMDHardwareInterface"),
          "Parameter 'homing_vel' not set for '%s' despite 'home_on_startup' being set true. It is required for homing procedure.", info_.joints[i].name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      else
      {
        motor_[i].homing_vel = std::stod(info_.joints[i].parameters["homing_vel"]);
        RCLCPP_INFO(
          rclcpp::get_logger(
            "RMDHardwareInterface"),
          "Parameter 'homing_vel' set for '%s' to %lf.",
          info_.joints[i].name.c_str(),motor_[i].homing_vel);
      }
    }
    else
    {
      motor_[i].homing_vel = 0.0;
    }
  }

  // Initialise Motor Params
  for (uint i = 0; i < info_.joints.size(); i++) {
    motor_[i].torque_constant = supported_motors_.at(motor_[i].model)[Params::torque_constant];
    motor_[i].max_velocity = supported_motors_.at(motor_[i].model)[Params::max_velocity];
    motor_[i].gear_ratio = supported_motors_.at(motor_[i].model)[Params::gear_ratio];
    motor_[i].range = supported_motors_.at(motor_[i].model)[Params::range];
  }

  activated = false;

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RMDHardwareInterface::on_configure(
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
  RCLCPP_INFO(rclcpp::get_logger("RMDHardwareInterface"), "Successfully configured!");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RMDHardwareInterface::export_state_interfaces()
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

std::vector<hardware_interface::CommandInterface> RMDHardwareInterface::export_command_interfaces()
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

hardware_interface::CallbackReturn RMDHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("RMDHardwareInterface"), "Activating RMDHardwareInterface");

  if (!can_intf_.init(
      can_interface_, &event_loop_,
      std::bind(&RMDHardwareInterface::recv_callback, this, _1)))
  {
    RCLCPP_INFO(
      rclcpp::get_logger(
        "RMDHardwareInterface"), "Failed to initialize socket can interface: %s",
      can_interface_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  can_event_loop = std::make_unique<std::thread>([&]() {event_loop_.run_until_empty();});

  for (uint i = 0; i < info_.joints.size(); i++)
  {
    request(&motor_[i],CommandIDs::motor_off);
    request(&motor_[i],CommandIDs::open_brake);
    request(&motor_[i],CommandIDs::motor_run);
    std::this_thread::sleep_for(std::chrono::seconds(5));

    if(motor_[i].home_on_startup)
    {
      send_vel(&motor_[i],motor_[i].homing_vel);
      while(!motor_[i].endstop_state)
      {
        request(&motor_[i],CommandIDs::read_motor_status_2);
      }
      motor_[i].homing_offset = motor_[i].raw_position_rad;
      send_vel(&motor_[i],0.0);
      motor_[i].homing_done = true;
    }
  }
  activated = true;
  return CallbackReturn::SUCCESS;
}

void RMDHardwareInterface::recv_callback(const can_frame & frame)
{
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    if(((frame.can_id) == 0x000) && (frame.len == 2) && (frame.data[0] == motor_[i].node_id))
    {
      motor_[i].endstop_state = bool(frame.data[1]);
      RCLCPP_INFO(
        rclcpp::get_logger("RMDHardwareInterface"), "'%s': Got Endstop as %d",info_.joints[i].name.c_str(),motor_[i].endstop_state);
      continue;
    }
    if((frame.can_id&0x1F) != motor_[i].node_id)
    {
      continue;
    }
    if (frame.len!=8)
    {
      RCLCPP_WARN(rclcpp::get_logger("RMDHardwareInterface"),
            "Incorrect frame length received from motor ID: %d. %d != 8", frame.can_id, frame.len);
      continue;
    }
    motor_[i].response_received = true;

    switch(frame.data[0])
    {
      case CommandIDs::read_pos_kp:
      break;
      case CommandIDs::read_pos_ki:
      break;
      case CommandIDs::read_vel_kp:
      break;
      case CommandIDs::read_vel_ki:
      break;
      case CommandIDs::read_torque_kp:
      break;
      case CommandIDs::read_torque_ki:
      break;
      case CommandIDs::read_acc:
      break;
      case CommandIDs::read_multiturn_counts:
      break;
      case CommandIDs::read_multiturn_counts_raw:
      break;
      case CommandIDs::read_multiturn_counts_offset:
      break;
      case CommandIDs::read_multiturn_angles:
      break;
      case CommandIDs::read_motor_status_1:
      break;
      case CommandIDs::read_motor_status_2:
        motor_[i].temperature = frame.data[0];
        motor_[i].raw_torque_n_m = map(read_le<int16_t>(frame.data + 2),-2048,2048,-33,33)*motor_[i].torque_constant;
        motor_[i].raw_velocity_rad_s = (read_le<int16_t>(frame.data + 4)/motor_[i].gear_ratio)*DEG2RAD;
        continuous_pos(&motor_[i],read_le<uint16_t>(frame.data + 6)/(motor_[i].gear_ratio*65535.0)*ROT2RAD);
      break;
      case CommandIDs::read_motor_status_3:
      break;
      case CommandIDs::torque_closed_loop:
        motor_[i].temperature = frame.data[0];
        motor_[i].raw_torque_n_m = map(read_le<int16_t>(frame.data + 2),-2048,2048,-33,33)*motor_[i].torque_constant;
        motor_[i].raw_velocity_rad_s = (read_le<int16_t>(frame.data + 4)/motor_[i].gear_ratio)*DEG2RAD;
        continuous_pos(&motor_[i],read_le<uint16_t>(frame.data + 6)/(motor_[i].gear_ratio*65535.0)*ROT2RAD);
      break;
      case CommandIDs::velocity_closed_loop:
        motor_[i].temperature = frame.data[0];
        motor_[i].raw_torque_n_m = map(read_le<int16_t>(frame.data + 2),-2048,2048,-33,33)*motor_[i].torque_constant;
        motor_[i].raw_velocity_rad_s = (read_le<int16_t>(frame.data + 4)/motor_[i].gear_ratio)*DEG2RAD;
        continuous_pos(&motor_[i],read_le<uint16_t>(frame.data + 6)/(motor_[i].gear_ratio*65535.0)*ROT2RAD);
      break;
      case CommandIDs::position_closed_loop_1:
      break;
      case CommandIDs::position_closed_loop_2:
      break;
      case CommandIDs::position_closed_loop_3:
      break;
      case CommandIDs::position_closed_loop_4:
      break;
      case CommandIDs::multiturn_incremental_pos:
      break;
      case CommandIDs::read_running_mode:
      break;
      case CommandIDs::read_power_status:
      break;
      case CommandIDs::read_battery_voltage:
      break;
      case CommandIDs::readwrite_can_id:
      break;
    }
  }
}

hardware_interface::CallbackReturn RMDHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RMDHardwareInterface"), "Stopping ...please wait...");
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    request(&motor_[i],CommandIDs::motor_off);
    request(&motor_[i],CommandIDs::close_brake);
  }

  can_intf_.deinit();

  // Ensure the thread is joined and resources are cleaned up
  if (can_event_loop->joinable()) {
    can_event_loop->join();
  }

  RCLCPP_INFO(rclcpp::get_logger("RMDHardwareInterface"), "System successfully stopped!");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type RMDHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    if(motor_[i].error_code!=0)
    {
      std::string error_message = error_codes_[motor_[i].error_code];
      RCLCPP_FATAL(
        rclcpp::get_logger(
          "RMDHardwareInterface"),
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

hardware_interface::return_type RMDHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if(!activated)
  {
    return hardware_interface::return_type::OK;
  }
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    // double position = ((motor_[i].hw_commands_positions_rad - motor_[i].offset) * motor_[i].reduction) - motor_[i].homing_offset;
    double velocity = motor_[i].hw_commands_velocities_rad_s * motor_[i].reduction;
    double torque = motor_[i].hw_commands_efforts_n_m / motor_[i].reduction;
    switch(motor_[i].control_mode)
    {
      case ControlModes::TORQUE:
      {
        send_torque(&motor_[i],torque);
      }
      break;
      case ControlModes::VELOCITY:
      {
        send_vel(&motor_[i],velocity);
      }
      break;
      case ControlModes::POSITION:
      {
      }
      break;
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace rmd_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  rmd_hardware_interface::RMDHardwareInterface, hardware_interface::ActuatorInterface)
