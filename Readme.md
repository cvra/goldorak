# Goldorak

## Setup of the Beaglebone black

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

Install some useful packages
```sh
sudo apt-get install git
```

## IO setup

To setup the IOs on the BBB, you can run the provided script
```sh
./sysops/goldorak_overlay.sh
```
