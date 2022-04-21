Configuring the Pi Station with Ansible
=======================================

[Ansible][ansible] is a configuration automation tool readily available on most Linux distribution;
e.g. on Debian:

``` bash
# Install Ansible
sudo apt-get install ansible
```

Or using a Python virtual environment:

``` bash
# Create an Ansible-dedicated Python virtual environment
virtualenv --python=/usr/bin/python3 "${HOME}/virtualenv/ansible"

# Activate the virtual environment
source "${HOME}/virtualenv/ansible/bin/activate"

# Install Ansible and other dependencies in the virtual environment
pip install -r ./requirements.txt

# Deactivate the virtual environment (when you're done with Ansible)
deactivate
```

[ansible]: https://www.ansible.com/


Pre-requisites
--------------

In order to configure the Pi Station with Ansible, you will need to:

* make sure you have password-less - key-based - [SSH access](../README.md#ssh-access) to the
  Raspberry Pi

* create your personal `inventory.yaml` inventory/preferences file - copy the `inventory.yaml.sample`
  one to get started - and make sure you list the _hostname(s)_ - e.g. `raspberrypi.example.org` -
  matching your Pi


How-to
------

Applying a configuration via Ansible is done by executing a _Playbook_, which applies given
_Role(s)_ to given _Hosts_ (group), defined in your `inventory.yaml`; example given, to display
the _Facts_ about your Pi:

``` bash
# Execute the `facts` playbook
ansible-playbook -i inventory.yaml playbook-facts.yaml

# OR (by targeting the role-specific playbook directly)
ansible-playbook -i inventory.yaml roles/facts/playbook.yaml
```


Roles and Playbooks
-------------------

### Roles

The following Roles are available to configure the Pi Station:

* `cleanup`: cleanup the stock installation, by removing unnecessary packages and fine-tuning the
  system to limit write operations that would wear the SD card out

* `hostname`: set the host name

* `bash`: configure the command-line wickedly for the nerdy sysadmin you are

* `ramoverlay`: setup a memory-based overlay over _specific_ well-known temporary directories (again,
  to limit write operations that would wear the SD card out)

* `ipv6`: enable/disable IPv6

* `iptables`: setup firewalling to protect your Internet-exposed Pi from network abuse

* `dhcpcd`: setup DHCP Client Daemon (custom configuration)

* `ethernet`: setup Ethernet, wired LAN (hardware)

* `wlan`: setup Wireless LAN (hardware and software dependencies)

* `huawei_hilink`: setup Huawei HiLink device (hardware and software dependencies)

* `rngd`: setup Random Number Generator (RNG) Daemon (software dependencies)

* `ntp`: setup Network Time (NTP) synchronization (software dependencies)

* `wireguard`: setup [WireGuard (VPN)][wireguard]

* `freedns`: setup [FreeDNS (dynamic DNS)][freedns] (specific) client

* `inadyn`: setup [In-a-Dyn (dynamic DNS)][inadyn] (generic) client

* `bluetooth`: setup Bluetooth (hardware and software dependencies)

* `audio`: setup audio (hardware and software dependencies)

* `v4l`: setup Video-4-Linux (V4L) cameras (hardware and software dependencies)

* `reboot`: schedule regular reboot

* `i2c`: setup the I2C bus (hardware and software dependencies)

* `pcf8523`: setup the PCF8523 Real-Time Clock (RTC)

* `sleepypi`: setup the Pi such as to work seamlessly with the [Sleepy Pi][sleepy-pi] power
  management hat

* `rtlsdr`: setup the Realtek RTL2832U Software Defined Radio (SDR)

* `ogn`: setup the [Open Glider Network (OGN)][ogn] receiver and decoder

[wireguard]: https://www.wireguard.com/
[freedns]: https://freedns.afraid.org/
[inadyn]: https://github.com/troglobit/inadyn
[sleepy-pi]: https://spellfoundry.com/product/sleepy-pi-2/
[ogn]: https://www.glidernet.org/

### Playbooks

Those can easily be played "as logical units" thanks to corresponding Playbooks:

* `playbook-bare`: clean up a vanilla Raspberry Pi installation and take it down to its bare
  minimum (spare resources - storage, RAM, CPU - as much as possible)

* `playbook-base`: install and configure the base/common components of the Pi Station

* `playbook-sleepypi`: install and configure [Sleepy Pi][sleepy-pi] components (incl. dependencies)

* `playbook-ogn`: install and configure [Open Glider Network (OGN)][ogn] components (incl. dependencies)

``` bash
# Execute all "base" roles
ansible-playbook -i inventory.yaml playbook-base.yaml
```
_NOTE:_ you can easily enable/disable each Role thanks to the corresponding `<ROLE>_ENABLE; {True|False}`
in the `inventory.yaml` file.

Or played individually:

``` bash
# Execute a given role
ansible-playbook -i inventory.yaml roles/<role>/playbook.yaml
```

### Testing (dry-run)

To test what a Role/Playbook would do _without_ actually applying its changes:

``` bash
# Test a playbook
ansible-playbook -i inventory.yaml roles/<role>/playbook.yaml --diff --check
```


Roles configuration
-------------------

Roles can have their configuration fine-tuned thanks to ad-hoc `<ROLE>_*` variables.

Refer to each Role's `roles/<role>/defaults/main.yaml` for further details on available variables
and their default values.


Raspberry Pi hardware configuration
-----------------------------------

Most of the Rabsberry Pi low-level hardware configuration is done in the `/boot/config.txt` file.

**Enabling/disabling** hardware components is achived via [Device Tree directives][rpi-config-dt] while
hardware **configuration** is done using ad-hoc [Configuration settings][rpi-config].

[rpi-config-dt]: https://www.raspberrypi.org/documentation/configuration/device-tree.md#part3
[rpi-config]: https://www.raspberrypi.org/documentation/configuration/config-txt/README.md
