# rocon_hardware_interfaces
ROS2 Control Hardware Interfaces for Research Development

## Contents
- [AK Hardware Interface](ak_hardware_interface)

## Instructions for checking the hardware interface
```bash
colcon build
source install/setup.bash
ros2 launch <hw_if_package_name> test.launch.py 
```
More info can be found in the readme of each package.

## Actual CAN Bus
```bash
sudo slcand -o -s8 -t sw -S 3000000 /dev/ttyUSB0 can0
sudo ifconfig can0 txqueuelen 1000
sudo ifconfig can0 up
```

## Virtual CAN Bus
```bash
sudo ip link add dev can0 type vcan
sudo ip link set up can0 txqueuelen 1000
sudo ip link show dev can0
```