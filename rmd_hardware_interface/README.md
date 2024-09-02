## CAN API
The RMD CAN API can be found [here](./misc/protocolV2_0.pdf). This is an extremely old protocol, and thus this hw_if does not work with newer versions. For newer version you can try [this](https://github.com/2b-t/myactuator_rmd_ros)

## ROS2 Control Tag
Info on some parameters is given below
- __Node_ID__: This is the Node ID of the RMD Motor. To be set using [RMD Actuator GUI](./misc/windows_setup.exe) (only on windows).
- __Model__: This is the model of the motor being used. Currently the following are accepted as valid inputs.
   - RMDX8_19
- __Control_Mode__: This is the control mode that the motor is to be run in. Following are valid inputs
   - torque
   - velocity (currently not supported)
   - position (currently not supported)
- __Offset__: This is the offset between the Joint's Zero and the Motor's Zero, and is expressed in Joint Space (in units of Joint Rotations[rads]). The following method can be used to calculate the offset for a system.
  - Setup your mechanical system and start RMD in any mode that allows you direct or indirect position control.
  - Bring the motor to its own zero.
  - Now rotate the motor (either manually or through RMD control) till the **Joint** reaches its origin
  - If the Raw Position Feedback from RMD(converted to radians) is taken as `pos` and the reduction between the systems is taken as `reduc`, the formula for the offset is as follows 
  $$-pos/reduc$$
   Note: The offset parameter is only useful if the RMD has the same starting position respective to the Joints Origin on every startup. This implies that you are getting absolute readings from your Motor that are persistent even on boot up.
- __home_on_startup__: Set to 'True' if you want to home on startup. Expects a endstop device that send a CAN message with following details.
  - ID = 0
  - LEN = 2
  - DATA[0] = Motor Node ID
  - DATA[1] = Endstop State (should be true when detected)
  Once Endstop send detected signal, the motor assumes that position to be zero. Offset from this zero can be set with the `Offset` param.
- __homing_vel__: Only used when `home_on_startup` is set to true. Sets vel that is used to move towards endstop. You can change the sign of the vel to change direction of seeking.

```bash
<ros2_control name="Wheel" type="actuator">
   <hardware>
      <plugin>rmd_hardware_interface/AkHardwareInterface</plugin>
      <param name="interface">can0</param>
   </hardware>
   <joint name="wheel_joint">
         <param name="node_id">2</param>
         <param name="model">RMDX8_19</param>
         <param name="control_mode">torque</param>
         <param name="reduction">1.0</param>
         <param name="offset">0.0</param>
         <param name="home_on_startup">1.0</param>
         <param name="homing_vel">1.0</param>
         <param name="offset">0.0</param>
         <state_interface name="position"/>
         <state_interface name="velocity"/>
         <command_interface name="effort"/>
   </joint>
</ros2_control>
```
## Prerequisites
- Latest Docker Images are pulled
- Latest Tested Stable Firmware is Flashed on RMD Hardware
- CANUSB is configured properly

## Steps to Test
- 
   ```bash
   ros2 launch rmd_hardware_interface test.launch.py
   ```
   If all is configured properly, moving motor shaft should result in movement of rectangular object in rviz. You can also check the wheel data on the joint states topic
   ```bash
   ros2 topic list
   ros2 topic echo /joint_states
   ```
- To send commands to the RMD you can use the following commands. Use the command for the controller that you have setup to use
  ```bash
  ros2 topic pub /effort_controller/commands std_msgs/msg/Float64MultiArray "data: [ 1.0 ]"
  ```

## Common Debugging Steps

1) Check if can interface is setup properly
   ```bash
   ip link ls
   ```
   This should show the can0 interface as up
2) Check if data is coming through CAN
   ```bash
   candump -c -ta -x can0
   ```
3) Check if Node ID is set properly in both RMD and URDF