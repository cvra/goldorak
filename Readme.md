# Project Goldorak

# Match checklist

1. Boot system (switch on 1)
2. SSH into the robot and start a tmux.
3. `roslaunch goldorak_bringup goldorak.launch`
4. Wait for `odom received`
5. Plug starter
6. Take reference by pushing on fishing module's Y and Z axis until you hear a click.
7. Open a new tmux pane (Ctrl-B then C).
8. Edit `smach_demo.py` to set team color.
9. `roslaunch goldorak_bringup strategy.launch`.
10. Wait for `waiting for starter` and then arm emergency stop.
11. Detach tmux (Ctrl-B then D), then exit SSH.
12. Pray and pull the starter when told to.

## Setup

### On your computer

You'll need to create a virtual CAN interface to simulate the CAN interface on the robot
```sh
fab vcan:'vcan0'
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
allow-hotplug can1

auto can1
iface can1 can static
	pre-up /usr/bin/env config-pin overlay cape-universaln
	pre-up /usr/bin/env config-pin P9.24 can
	pre-up /usr/bin/env config-pin P9.26 can

    bitrate 1000000
    samplepoint 0.875
```

## Build & Run

Build the nodes and run with simulated motors
```sh
./build.sh
roslaunch goldorak_bringup goldorak.launch simulated_motors:=true
```

Then, you can give target points to the robot via the monitoring interface
```sh
roslaunch goldorak_bringup monitor.launch
```

## Simulation (Gazebo)

The robot can also be simulated using Gazebo.
In this case, the differential base control and odometry are handled by ROS Control.
Higher level nodes will run as on the robot.

Run the simulation
```sh
roslaunch goldorak_simulation goldorak.launch
```

## Simulation (ROS only)

The robot can also be simulated partially without Gazebo.
Motor boards are simulated by a basic python script.
Higher level nodes will run as on the robot.

Run the simulation
```sh
roslaunch goldorak_bringup goldorak.launch simulated_motors:=true
```

## Dependencies

For dependencies, you should look at the setup guidelines in `Setup.md`.
