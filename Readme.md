# Project Goldorak

## Setup

### On your computer

You'll need to create a virtual CAN interface to simulate the CAN interface on the robot
```sh
./sysops/create_virtual_can.sh vcan0
```

### On the robot / the BBB

You'll need to setup the CAN interface on the BBB of the robot
```sh
./sysops/goldorak_overlay.sh
```

## Build & Run

Build the UAVCAN stack, DSDL messages, ROS package and run it
```sh
./build.sh
rosrun uavcan_core uavcan_bridge 10 # UAVCAN node ID 10
```
