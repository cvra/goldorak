# Project Goldorak

## Setup

### On your computer

You'll need to create a virtual CAN interface to simulate the CAN interface on the robot
```sh
./sysops/create_virtual_can.sh vcan0
```

### On the robot / the BBB

Append the following to `/etc/modules`:

```
can
can_raw
vcan
```

Append the following to `/etc/network/interfaces`:

```
auto can1
iface can1 can static
	pre-up /usr/bin/env config-pin overlay cape-universaln
	pre-up /usr/bin/env config-pin P9.24 can
	pre-up /usr/bin/env config-pin P9.26 can

    bitrate 1000000
    samplepoint 0.875
```

## Build & Run

Build the UAVCAN stack, DSDL messages, ROS package and run it
```sh
./build.sh
rosrun uavcan_core uavcan_bridge 10 # UAVCAN node ID 10
```
