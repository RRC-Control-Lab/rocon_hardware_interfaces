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

  for (uint i = 0; i < info_.joints.size(); i++) {
    it = info_.joints[i].parameters.find("node_id");
    if (it == info_.joints[i].parameters.end()) {
      RCLCPP_FATAL(
        rclcpp::get_logger(
          "AKHardwareInterface"),
        "Parameter 'node_id' not set for '%s'. It is required for Socket CAN.",
        info_.joints[i].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    } else {
      motor_[i].node_id = std::stoi(it->second);
    }
    it = info_.joints[i].parameters.find("model");
    if (it == info_.joints[i].parameters.end()) {
      RCLCPP_FATAL(
        rclcpp::get_logger(
          "AKHardwareInterface"),
        "Parameter 'model' not set for '%s'. Valid values are AK80_9, AK10_9, AK60_6, AK70_10, AK80_6, AK80_64.",
        info_.joints[i].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    } else {
      motor_[i].model = it->second;
    }
    it = info_.joints[i].parameters.find("control_mode");
    if (it == info_.joints[i].parameters.end()) {
      RCLCPP_FATAL(
        rclcpp::get_logger(
          "AKHardwareInterface"),
        "Parameter 'control_mode' not set for '%s'. Valid values are torque, velocity, position.",
        info_.joints[i].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    } else {
      motor_[i].control_mode = control_modes.at(it->second);
    }
    it = info_.joints[i].parameters.find("reduction");
    if (it == info_.joints[i].parameters.end()) {
      RCLCPP_INFO(
        rclcpp::get_logger(
          "AKHardwareInterface"),
        "Parameter 'reduction' not set for '%s'. Defaulting to 1.0.",
        info_.joints[i].name.c_str());
      motor_[i].reduction = 1.0;
    } else {
      motor_[i].reduction = std::stod(it->second);
      RCLCPP_INFO(
        rclcpp::get_logger(
          "AKHardwareInterface"),
        "Parameter 'reduction' set for '%s' to %lf.",
        info_.joints[i].name.c_str(),motor_[i].reduction);
    }
    it = info_.joints[i].parameters.find("offset");
    if (it == info_.joints[i].parameters.end()) {
      RCLCPP_INFO(
        rclcpp::get_logger(
          "AKHardwareInterface"),
        "Parameter 'offset' not set for '%s'. Defaulting to 0.0.",
        info_.joints[i].name.c_str());
      motor_[i].offset = 0.0;
    } else {
      motor_[i].offset = std::stod(it->second);
      RCLCPP_INFO(
        rclcpp::get_logger(
          "AKHardwareInterface"),
        "Parameter 'offset' set for '%s' to %lf.",
        info_.joints[i].name.c_str(),motor_[i].offset);
    }
    it = info_.joints[i].parameters.find("kp");
    if (it == info_.joints[i].parameters.end()) {
      RCLCPP_INFO(
        rclcpp::get_logger(
          "AKHardwareInterface"),
        "Parameter 'kp' not set for '%s'. Defaulting to 0.0.",
        info_.joints[i].name.c_str());
      motor_[i].kp = 0.0;
    } else {
      motor_[i].kp = std::stod(it->second);
      RCLCPP_INFO(
        rclcpp::get_logger(
          "AKHardwareInterface"),
        "Parameter 'kp' set for '%s' to %lf.",
        info_.joints[i].name.c_str(),motor_[i].kp);
    }
    it = info_.joints[i].parameters.find("kd");
    if (it == info_.joints[i].parameters.end()) {
      RCLCPP_INFO(
        rclcpp::get_logger(
          "AKHardwareInterface"),
        "Parameter 'kd' not set for '%s'. Defaulting to 0.0.",
        info_.joints[i].name.c_str());
      motor_[i].kd = 0.0;
    } else {
      motor_[i].kd = std::stod(it->second);
      RCLCPP_INFO(
        rclcpp::get_logger(
          "AKHardwareInterface"),
        "Parameter 'kd' set for '%s' to %lf.",
        info_.joints[i].name.c_str(),motor_[i].kd);
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
    motor_[i].raw_position_rad = motor_[i].wrap_offset + motor_[i].curr_wrap_position_rad;
  }
}

hardware_interface::CallbackReturn AKHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("AKHardwareInterface"), "Stopping ...please wait...");

  struct can_frame frame;
  frame.len = 8;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    frame.can_id = motor_[i].node_id;

    /// convert floats to unsigned ints ///
    uint16_t p_int = float_to_uint(0.0, motor_[i].P_min, motor_[i].P_max, 16);
    uint16_t v_int = float_to_uint(0.0, motor_[i].V_min, motor_[i].V_max, 12);
    uint16_t kp_int = float_to_uint(0.0, motor_[i].Kp_min, motor_[i].Kp_max, 12);
    uint16_t kd_int = float_to_uint(0.0, motor_[i].Kd_min, motor_[i].Kd_max, 12);
    uint16_t t_int = float_to_uint(0.0, motor_[i].T_min, motor_[i].T_max, 12);
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

    const uint8_t data_values[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    std::copy(data_values, data_values + 8, frame.data);
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
  struct can_frame frame;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    frame = can_frame{};
    frame.can_id = motor_[i].node_id;
    frame.len = 8;
    double p_des = (motor_[i].hw_commands_positions_rad - motor_[i].offset) * motor_[i].reduction;
    double v_des = motor_[i].hw_commands_velocities_rad_s * motor_[i].reduction;
    double t_ff = motor_[i].hw_commands_efforts_n_m / motor_[i].reduction;
    double kp = motor_[i].kp;
    double kd = motor_[i].kd;
    switch(motor_[i].control_mode)
    {
      case ControlModes::TORQUE:
      {
        p_des = 0;
        v_des = 0;
        kp = 0;
        kd = 0;
        t_ff = fminf(fmaxf(motor_[i].T_min, motor_[i].hw_commands_efforts_n_m), motor_[i].T_max);
      }
      break;
      case ControlModes::VELOCITY:
      {
        p_des = 0;
        v_des = fminf(fmaxf(motor_[i].V_min, motor_[i].hw_commands_velocities_rad_s), motor_[i].V_max);
        kp = 0;
        kd = fminf(fmaxf(motor_[i].Kd_min, kd), motor_[i].Kd_max);
        t_ff = 0;
      }
      break;
      case ControlModes::POSITION:
      {
        p_des = fminf(fmaxf(motor_[i].P_min, motor_[i].hw_commands_positions_rad), motor_[i].P_max);
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
