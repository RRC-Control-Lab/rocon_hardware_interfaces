// Copyright (c) 2024, Soham Patil
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include <gmock/gmock.h>

#include <string>

#include "hardware_interface/resource_manager.hpp"
#include "ros2_control_test_assets/components_urdfs.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

class TestOdriveHardwareInterface : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // TODO(anyone): Extend this description to your robot
    odrive_hardware_interface_2dof_ =
      R"(
        <ros2_control name="OdriveHardwareInterface2dof" type="actuator">
          <hardware>
            <plugin>odrive_hardware_interface/OdriveHardwareInterface</plugin>
          </hardware>
          <joint name="joint1">
            <command_interface name="position"/>
            <state_interface name="position"/>
            <param name="initial_position">1.57</param>
          </joint>
          <joint name="joint2">
            <command_interface name="position"/>
            <state_interface name="position"/>
            <param name="initial_position">0.7854</param>
          </joint>
        </ros2_control>
    )";
  }

  std::string odrive_hardware_interface_2dof_;
};

TEST_F(TestOdriveHardwareInterface, load_odrive_hardware_interface_2dof)
{
  auto urdf = ros2_control_test_assets::urdf_head + odrive_hardware_interface_2dof_ +
              ros2_control_test_assets::urdf_tail;
  ASSERT_NO_THROW(hardware_interface::ResourceManager rm(urdf));
}
