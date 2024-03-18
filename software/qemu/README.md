QEMU-lated Raspberry Pi
=======================

The [QEMU][qemu] emulator (for ARM architectures) is readily available on most Linux distribution;
e.g. on Debian:

``` bash
# Install QEMU
sudo apt-get install qemu-system-arm
```

[qemu]: https://www.qemu.org/


How-to
------

Emulating the Raspberry PI with QEMU requires to:

* Download the latest Raspberry Pi OS version - **Lite** edition - from the
  [Raspberry Pi OS download][rpi-os-download]

* Extract and resize the Raspberry Pi OS image from that download

* Extract the Linux _kernel_ and hardware _DTB (Device Tree Binary)_ files
  from that image

* Inject a few "firstboot" parameters in that image, like pre-configuring
  the Raspberry Pi's username and password and enabling SSH

* Launch the QEMU emulator with the proper parameters

All of it being handily automated in the [qemu-raspi3b](./qemu-raspi3b) script:

``` bash
# Emulate the Raspberry PI with QEMU
./qemu-raspi3b
# [output]
#NOTICE[qemu-raspi3b]: Downloading Raspberry PI OS image
#Saving to: ‘./data/2024-03-15-raspios-bookworm-arm64-lite.img.xz’
#2024-03-15-raspios-bookworm-arm64-lite.img.xz’ saved [434372360/434372360]
#NOTICE[qemu-raspi3b]: Creating the Raspberry PI OS image
#NOTICE[qemu-raspi3b]: Mounting the Raspberry PI OS boot partition
#NOTICE[qemu-raspi3b]: Fine-tuning the Raspberry PI OS boot configuration: kernel parameters
#NOTICE[qemu-raspi3b]: Fine-tuning the Raspberry PI OS boot configuration: hardware configuration
#NOTICE[qemu-raspi3b]: Fine-tuning the Raspberry PI OS boot configuration: user
#NOTICE[qemu-raspi3b]: Extracting the Raspberry PI OS kernel and DTB
#NOTICE[qemu-raspi3b]: Unmounting the Raspberry PI OS boot partition
#WARNING[qemu-raspi3b]: This is the first time this Raspiberry PI OS image boots:
#WARNING[qemu-raspi3b]: Once initialized, the QEMU boot directory will be updated (re-created)
#PROMPT[qemu-raspi3b] Use the stock DTB (instead of the QEMU-specific one) [yes/No] ?
#PROMPT[qemu-raspi3b] Skip launching the Raspberry PI OS QEMU-lation now [yes/No] ?
#NOTICE[qemu-raspi3b]: Launching the Rapsberry PI OS QEMU-lation; you may now:
#NOTICE[qemu-raspi3b]: - use <CTRL+A>+<C> to enter the QEMU prompt
#NOTICE[qemu-raspi3b]: - use VNC to access the console; e.g. gvncviewer 127.0.0.1:0
#NOTICE[qemu-raspi3b]: - use SSH to access the command prompt; e.g. ssh -p 2222 pi@127.0.0.1
```

NOTA BENE: The downloaded and extracted Raspberry Pi OS will be located in `data` sub-directory

[rpi-os-download]: https://www.raspberrypi.com/software/operating-systems/


Customized DTB
--------------

In order to have the console output redirected to the serial line (QEMU's `-serial stdio`), one must
compile a customized DTB, which disables Bluetooth and enables the (QEMU-lated) Broadcom 2835 UART0
serial hardware. Have a look in the `dtb` sub-directory.

This customized DTB is most easily compiled _in the (QEMU-lated) Raspberry PI_ itself, using the
`dtmerge` tool (included in the Raspberry PI OS image):

``` bash
# Compile a customized DTB
cp /boot/firmware/bcm2710-rpi-3-b-plus.dtb custom.dtb
# (dtparam=uart0=on)
dtmerge custom.dtb merged.dtb - uart0=on
mv merged.dtb custom.dtb
# (dtoverlay=disable-bt)
dtmerge custom.dtb merged.dtb /boot/firmware/overlays/disable-bt.dtbo
mv merged.dtb custom.dtb
```
