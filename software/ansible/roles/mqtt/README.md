Ansible role `mqtt` - MQTT setup
=====

This role installs and pre-configures the various MQTT components (applications) -
[Mosquitto MQTT broker][mosquitto] and/or [Zigbee2MQTT bridge][zigbee2mqtt] -
via [Docker Compose][docker-compose].

[mosquitto]: https://mosquitto.org/
[zigbee2mqtt]: https://www.zigbee2mqtt.io/
[docker-compose]: https://docs.docker.com/compose/

In order for Ansible to manage those Docker Compose'd applications, one must add
the corresponding `docker` role stanza:

``` yaml
DOCKER_ENABLE: true
DOCKER_NETWORKS:
    "mqtt":
    ipam_config:
        - subnet: "192.168.183.0/24"
          gateway: "192.168.183.1"
DOCKER_COMPOSE_APPS:
  "mqtt":
    manage: false  # set to true after Ansible has also passed the `mqtt` role
    project_src: "/opt/mqtt"
    environment:
      COMPOSE_PROFILES: "mosquitto,zigbee2mqtt"
      ZIGBEE2MQTT_DEVICE: "/dev/serial/by-id/..."
      # (see the docker-compose.yaml file for other environment variables)
```


WARNING - Use a proper power supply!
-----

The power drawn by Zigbee adapters will lead to unstable operations of both
Raspberry Pi and Zigbee if using a weak power supply; tell-tale signs will be:
- `Undervoltage detected` messages in `dmesg`
- `Adapter disconnected` messages in `Zigbee2MQTT`
- Raspberry Pi reboots

Be aware that phone chargers, even the ones capable of Quick/Fast Charge (with
max. current rated well above 2.5A at 5V), do NOT necessarily provide a stable,
uninterrupted 5V output at all time. It is thus HIGHLY recommended to use a
Raspberry Pi-specific power supply (or a Sleepy Pi equivalent).


WARNING - Incompatible kernel (>= 6.6)
-----

**WARNING: The procedure below applies to a Raspberry Pi 3B+ (64-bit OS).
If using another version, make sure to `rpi-update` the proper bootloader,
kernel and initramfs!**

As of April 2025, if using Raspberry Pi Linux kernel equal or above 6.6.x,
you _may_ experience the Pi freezing when Zigbee2MQTT starts - see
[GitHub issue #22881][zigbee2mqtt-issue-22881] - and need to downgrade the
kernel to 6.1.77 (the latest in the 6.1 serie) using the `rpi-update` tool.

[zigbee2mqtt-issue-22881]: https://github.com/Koenkk/zigbee2mqtt/issues/22881

Here be dragons...


``` shell
# Install dependencies
apt-get update
apt-get install rpi-update

# Install a "placeholder" kernel; this kernel will NOT be used
# (such as to not break modules dependencies; e.g. wireguard-modules)
apt-get install raspberrypi-kernel

# Remove "stock" kernel(s) FIRST
# (prevent automatic updates)
apt-get autoremove --purge linux-image-*

# Remove "stock" firmware SECOND
# (prevent automatic updates)
apt-get autoremove --purge raspi-firmware

# Manually install 6.1.77 kernel
rm -v /boot/firmware/.*_revision
WANT_64BIT=1 rpi-update 5fc4f643d2e9c5aa972828705a902d184527ae3f

# Verify the installed kernel matches the 6.1.77's SHA256 signature
sha256sum /boot/firmware/kernel8.img
: [output]
: 51c21fe38cd38ee55ab3f35271444b1878754ef17a0036331a1863f2c7c3c90d  /boot/firmware/kernel8.img
```

NOTE: If needs be, check the commits in the [rpi-firmware GitHub repository][rpi-firmware-commits]
to find the SHA hash for another kernel version.

[rpi-firmware-commits]: https://github.com/raspberrypi/rpi-firmware/commits/master/
