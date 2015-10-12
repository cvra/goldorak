#!/bin/bash

sudo modprobe can
sudo modprobe can_raw
sudo modprobe can_bcm
sudo modprobe vcan

sudo ip link add dev $1 type vcan
sudo ip link set up $1

sudo ifconfig $1 up
