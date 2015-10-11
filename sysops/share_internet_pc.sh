#!/bin/bash
sudo iptables -A POSTROUTING -t nat -j MASQUERADE
sudo echo 1 | sudo tee /proc/sys/net/ipv4/ip_forward > /dev/null
