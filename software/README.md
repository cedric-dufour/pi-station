Raspberry Pi Station - Software
===============================

**WARNING: The Raspberry Pi Station is installed with the [Raspberry PI OS _64-bit_](https://www.raspberrypi.com/news/raspberry-pi-os-64-bit/),
which requires a Raspberry PI 3 (or above)**

Download and Install the OS
---------------------------

On a workstation/laptop:

* Download the latest Raspberry Pi OS version - **Lite** edition - from the
  [Raspberry Pi OS download][rpi-os-download] page:

``` bash
# Download
wget https://downloads.raspberrypi.com/raspios_lite_arm64/images/raspios_lite_arm64-2024-03-15/2024-03-15-raspios-bookworm-arm64-lite.img.xz
ln -s 2024-03-15-raspios-bookworm-arm64-lite.img.xz raspios.img.xz
```

* Insert the SD card and mark its device identifier (`sdX`):

``` bash
# Find the inserted SD card device identifier
sudo dmesg | tail -n 25
# [output]
# ... scsi 12:0:0:0: Direct-Access     Generic  STORAGE DEVICE   0545 PQ: 0 ANSI: 0
# ... sd 12:0:0:0: Attached scsi generic sg6 type 0
# ... sd 12:0:0:0: [sdX] 30619648 512-byte logical blocks: (15.7 GB/14.6 GiB)
# ... sd 12:0:0:0: [sdX] Write Protect is off
# ... sd 12:0:0:0: [sdX] Mode Sense: 0b 00 00 08
# ... sd 12:0:0:0: [sdX] No Caching mode page found
# ... sd 12:0:0:0: [sdX] Assuming drive cache: write through
```

* Install the OS on the SD card (`/dev/sdX`):

```
# Install the image to the SD card
xzcat raspios.img.xz | sudo dd of=/dev/sdX bs=4M status=progress
# [output]
#2768240640 bytes (2.8 GB, 2.6 GiB) copied, 44.0475 s, 62.8 MB/s
```

[rpi-os-download]: https://www.raspberrypi.com/software/operating-systems/


Bootstrap
---------

* Insert the SD card into the Raspberry Pi and connect both a keyboard and (HDMI) screen to it
  before switching the power on

* Switch the power on

* Follow the firstboot wizard:
  - choose your keyboard layout
  - specify the Raspberry Pi's username and password (e.g. `pi`/`raspberry`)

* Configure the WLAN connection:

``` bash
# Start the Raspberry Pi configuration utility
sudo raspi-config

# Set the Wireless LAN
# > 1 System Options > S1 Wireless LAN

# Verify an IP address is obtained (via DHCP)
ip addr show dev wlan0
# [output]
#inet 192.168.1.101/24 brd 192.168.1.255 scope global dynamic noprefixroute wlan0
```

* Configure the timezone:

``` bash
# Change the timezone
# (recommended: UTC)
sudo dpkg-reconfigure tzdata
```

SSH Access
----------

* Enable and start SSH:

``` bash
# Enable and start the server
sudo systemctl enable ssh
sudo systemctl start ssh

# Verify the SSH server is enabed and running
sudo systemctl status ssh
# [output]
#ssh.service - OpenBSD Secure Shell server
#  Loaded: loaded (/lib/systemd/system/ssh.service; enabled; preset: enabled)
#  Active: active (running) since ...
```

* Install your [SSH (public) key][ssh-key]:

``` bash
## [on your workstation/laptop]

# Create an SSH key-pair (IF YOU DO NOT ALREADY HAVE ONE)
# WARNING: DON'T LOOSE IT! AND DO USE A PASSWORD!
ssh-keygen -t ed25519 -C "$(whoami)@$(hostname -f)"

# Display the SSH (public) key
cat ~/.ssh/id_ed25519.pub
# [output]
#<type>       <key>          <comment>
#ssh-ed25519 AAAAC3NzaC... jane@workstation.example.org

## [on the Pi]

# Add the SSH (public) key to the authorized keys list
mkdir -p /home/pi/.ssh
echo 'ssh-...' \
>> /home/pi/.ssh/authorized_keys
```

[ssh-key]: https://www.ssh.com/ssh/key/


Configuration using Ansible
---------------------------

The rest of the configuration will be carried out with [Ansible][ansible].

[ansible]: https://www.ansible.com/

Please see the ad-hoc [README](./ansible/README.md) to continue.


Power Management using Sleepy Pi
--------------------------------

The [Sleepy Pi][sleepy-pi] is a power management hat that allows the Pi Station to be powered
directly by the _external_ 12Volt power supply and sport advanced power management to optimize
its power consumption.

[sleepy-pi]: https://spellfoundry.com/product/sleepy-pi-2/

The Sleepy Pi comes from factory with a default program that features only _push button_ power
control; it needs to be re-programmed to provide the advanced power management abilities we
need.

Please see the ad-hoc [README](./sleepy-pi/README.md) to continue.


QEMU-lated Raspberry Pi (DEVELOPERS ONLY)
-----------------------------------------

The general-purpose [QEMU][qemu] emulator may come handy, should you want to test things out without
having the hardware Raspberry Pi at hand.

[qemu]: https://www.qemu.org/

Please see the ad-hoc [README](./qemu/README.md) to continue.
