Raspberry Pi Station - Software
===============================

Download and Install the OS
---------------------------

On a workstation/laptop:

* Download the latest Raspberry Pi OS version - **Lite** edition - from the
  [Raspberry Pi OS download][rpi-os-download] page:

``` bash
# Download
wget https://downloads.raspberrypi.org/raspios_lite_arm64/images/raspios_lite_arm64-2021-11-08/2021-10-30-raspios-bullseye-arm64-lite.zip
ln -s 2021-10-30-raspios-bullseye-arm64-lite.zip raspios.zip
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
# Retrieve the actual image file name
RPI_IMG="$(unzip -qqv raspios.zip '*.img' | sed -nE 's|^.*\s+(\S+\.img)|\1|p')"
echo "${RPI_IMG}"

# Install the image to the SD card
unzip -p raspios.zip "${RPI_IMG}" | sudo dd of=/dev/sdX bs=4M status=progress
# [output]
#1845493760 bytes (1.8 GB, 1.7 GiB) copied, 74.0463 s, 24.9 MB/s
```

[rpi-os-download]: https://www.raspberrypi.org/downloads/raspberry-pi-os/


Bootstrap
---------

* Insert the SD card into the Raspberry Pi and connect both a keyboard and (HDMI) screen to it
  before switching the power on

* Switch the power on

* Login once the boot has completed (username: `pi`; password: `raspberry`)

* **CHANGE THE DEFAULT PASSWORD !!!**

``` bash
# Start the Raspberry Pi configuration utility
sudo raspi-config

# Change user password
# > 1 System Options > S3 Password
```

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

* Install and enable SSH:

``` bash
# Update APT software repository
sudo agt-get update

# Install the OpenSSH server
sudo apt-get install --no-install-recommends openssh-server

# Enable and start the server
sudo systemctl enable ssh
sudo systemctl start ssh
```

* Install your [SSH (public) key][ssh-key]:

``` bash
## [on your workstation/laptop]

# Create an SSH key-pair (if you do not already have one)
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
