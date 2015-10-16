#!/bin/bash
sudo /sbin/route add default gw 192.168.7.1
sudo echo "nameserver 8.8.8.8" >> /etc/resolv.conf

echo "Updating system time"
sudo ntpdate -b -s -u pool.ntp.org
