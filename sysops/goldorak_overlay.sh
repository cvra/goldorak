#!/bin/bash

config-pin overlay cape-universaln

# CAN
sudo config-pin P9.24 can
sudo config-pin P9.26 can

# SocketCAN
sudo ip link set can1 up type can bitrate 1000000 sample-point 0.875 restart-ms 100
sudo ifconfig can1 up
