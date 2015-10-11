# Goldorak

## Setup of the Beaglebone black

First, get an SD card (at least 4GB) that we'll use to flash the Beaglebone black.
Then, download an Ubuntu 14.04 image from the [official links](https://rcn-ee.com/rootfs/),
choose a flasher image and then copy it on the SD card:
```sh
wget https://rcn-ee.com/rootfs/2015-05-08/flasher/BBB-eMMC-flasher-ubuntu-14.04.3-console-armhf-2015-10-09-2gb.img.xz
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
