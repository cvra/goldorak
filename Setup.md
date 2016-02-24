# Setup of the Beaglebone black

### Ubuntu & Xenomai

First, get an SD card (at least 4GB) that we'll use to flash the Beaglebone black.
Then, download an Ubuntu 14.04 image from the [official links](https://rcn-ee.com/rootfs/),
choose a flasher image and then copy it on the SD card:
```sh
wget https://rcn-ee.com/rootfs/2015-10-09/flasher/BBB-eMMC-flasher-ubuntu-14.04.3-console-armhf-2015-10-09-2gb.img.xz
unxz BBB-eMMC-flasher-ubuntu-14.04*.img.xz
sudo dd if=./BBB-eMMC-flasher-ubuntu-14.04*.img of=/dev/mmcblk0
```

Remove the SD card from your computer and plug it in the Beaglebone.
Press the S2 button while powering it up so it boots from the SD card.
It will start flashing the image to the eMMC.
After a few seconds, the LEDs should start in a Cylon pattern.
Once the LEDs are off, it’s done, you can unplug/replug it and SSH into it.
```sh
ssh ubuntu@192.168.7.2
```

Share internet using this on you PC
```sh
./sysops/share_internet_pc.sh
```

And this on your BBB
```sh
./sysops/share_internet_bbb.sh
```

Then, we install Xenomai
```sh
sudo apt-get update
sudo apt-get install linux-image-3.8.13-xenomai-r78 linux-headers-3.8.13-xenomai-r78 linux-firmware-image-3.8.13-xenomai-r78
sudo reboot
```

Then SSH again and install userspace bindings of Xenomai
```sh
wget http://download.gna.org/xenomai/stable/xenomai-2.6.4.tar.bz2
tar xvjf xenomai-2.6.4.tar.bz2

cd xenomai-2.6.4
./configure
make
sudo make install
```

You could check by running some example
```sh
sudo /usr/xenomai/bin/latency
```

Time to setup the IOs
```sh
sudo -s

apt-get install device-tree-compiler -y

cd /opt/source
git clone https://github.com/cdsteinkuehler/beaglebone-universal-io

cd beaglebone-universal-io
make install

exit
```

### CAN setup

Start by getting can-utils
```sh
sudo apt-get install git dh-autoreconf

git clone https://github.com/linux-can/can-utils.git
cd can-utils

./autogen.sh
./configure
make
sudo checkinstall make install
```

Now, we need UAVCAN in order to use the command line utilities
```sh
sudo apt-get install cmake
git clone https://github.com/UAVCAN/libuavcan

cd libuavcan
git submodule update --init

mkdir build
cd build
cmake ..
make
```

### IOs setup

To setup the IOs on the BBB, you can run the provided script
```sh
./sysops/goldorak_overlay.sh
```

### ROS installation

```sh
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -

sudo apt-get update
sudo apt-get install ros-indigo-ros-base

sudo apt-get install python-rosdep
sudo rosdep init
rosdep update

echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "export DISTRIB_ID=Ubuntu" >> ~/.bashrc
echo "export DISTRIB_RELEASE=14.04" >> ~/.bashrc
echo "export DISTRIB_CODENAME=trusty" >> ~/.bashrc
echo "export DISTRIB_DESCRIPTION="Ubuntu 14.04"" >> ~/.bashrc

sudo apt-get install python-rosinstall python-catkin-tools
sudo apt-get install ros-indigo-navigation ros-indigo-xacro ros-indigo-robot-state-publisher
sudo apt-get upgrade
```

### Catkin workspace setup

```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace

cd ~/catkin_ws
catkin_make
rm -rf build/ devel/
```
