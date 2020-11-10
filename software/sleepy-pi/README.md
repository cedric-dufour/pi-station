Advanced Power Management using the Sleepy Pi
=============================================

The [Sleepy Pi][sleepy-pi] is a power management hat that has several nifty features:

* it can be powered by an _external_ source with a voltage ranging from **5.5 to 30 Volt**

* it allows to gracefully shut down and power up the Raspberry Pi, manually using its _push button_
  or _programatically_, e.g. along its onboard, battery-backed Real-Time Clock (RTC)

* when shut down (yet standing by), power consumption is reduced to a minimum

* all of it thanks ot the integrated [Arduino][arduino], a 3.3Volt, 8Mhz [ATmega328P][atmega328p]
  microcontroller which can be programmed for whichever and however fancy power management strategy
  one may deem fit

[sleepy-pi]: https://spellfoundry.com/product/sleepy-pi-2/
[arduino]: https://www.arduino.cc/
[atmega328p]: https://www.microchip.com/wwwproducts/en/ATMEGA328P

The Sleepy Pi comes from factory with a default program that features only _push button_ power
control; it needs to be re-programmed to provide the advanced power management abilities we
need.


Programming Cable
-----------------

Although the Sleepy Pi may be programmed directly from the Raspberry Pi, it is better recommended to
do so from your workstation/laptop such as:
- keep the Pi Station environment as lean as possible
- benefit from your much more powerful workstation/laptop
- avoid messing with the Sleepy Pi jumpers
  (to keep the Raspberry Pi from shutting down when programming the Sleepy Pi/Arduino)

To do so, you will need a **3.3 Volt** USB-to-Serial programming cable, such as the one readily
available from [SpeelFoundry][sleepy-pi-cable].

[sleepy-pi-cable]: https://spellfoundry.com/product/sleepy-pi-external-programming-adapter-console/


Install the Arduino IDE
-----------------------

In order to re-program the Sleepy Pi, one needs the [Arduino Integrated Development Environment (IDE)][arduino-ide].

[arduino-ide]: https://www.arduino.cc/en/Main/Software

(Debian users may want to have a look at my [Arduino Debian packaging resource][arduino-debian] if
they want to install the IDE from a Debian package instead of manually)

[arduino-debian]: https://github.com/cedric-dufour/debian/tree/master/all/arduino


Install the Sleepy Pi dependencies
----------------------------------

Once the Arduino IDE installed, you will need to download and install the Sleepy Pi dependencies
(hardware specification and libraries).

_NOTE:_ Those instructions are merely a copy-paste of [upstream setup script][sleepy-pi-setup].
Also note that most can be downloaded within the IDE: `[Menu] > Tools > Manage Libraries...`.

[sleepy-pi-setup]: https://github.com/SpellFoundry/Sleepy-Pi-Setup/blob/master/Sleepy-Pi-Setup.sh

Assuming your Arduino environment lives in `~/Arduino`, prepare the required directories:

``` bash
# Arduino IDE libraries folder
mkdir -p ~/Arduino/libraries
```

### Sleepy PI

``` bash
# Sleepy Pi hardware specification
if ! grep -qF sleepypi /opt/arduino/hardware/arduino/avr/boards.txt; then
  cat ./boards.txt | sudo tee -a /opt/arduino/hardware/arduino/avr/boards.txt
fi

# Sleepy Pi (1) library
git clone https://github.com/SpellFoundry/SleepyPi.git ~/Arduino/libraries/SleepyPi

# Sleepy Pi 2 library
git clone https://github.com/SpellFoundry/SleepyPi2.git ~/Arduino/libraries/SleepyPi2
```

### PCF8523 - Real-Time Clock (RTC)

``` bash
# PCF8523 library
git clone https://github.com/SpellFoundry/PCF8523.git ~/Arduino/libraries/PCF8523
```

### Other dependencies

``` bash
# Time library
git clone https://github.com/PaulStoffregen/Time.git ~/Arduino/libraries/Time

# Low-Power
git clone https://github.com/rocketscream/Low-Power.git ~/Arduino/libraries/LowPower

# PinChangeInt library
git clone https://github.com/GreyGnome/PinChangeInt.git ~/Arduino/libraries/PinChangeInt
```

### Verify the IDE setup

Have a look at the [official Sleepy Pi documentation][sleepy-pi-ide] to verify the environment is
sucessfully set up.

[sleepy-pi-ide]: https://spellfoundry.com/docs/programming-from-the-arduino-ide/


Reprogram the Sleepy Pi
-----------------------

Now that the Arduino IDE is ready, you will just need to:

* Create a new _Sketch_ (`[Menu] > File > New`)

* Copy the content from the [sleepy-pi.ino](./sleepy-pi.ino) file

* Make sure to use the Sleepy Pi _Board_ (`[Menu] > Tools > Board > Sleepy Pi`)

* _Verify/Compile_ the new program (`[Menu] > Sketch > Verify/Compile`)

* _Upload_ the new program to the Sleepy Pi (`[Menu] > Sketch > Upload`);
  **WARNING:** This will cut the power the Raspberry Pi (ungracefully)!

### Sleepy Pi Programming Options

The following options are available to fine-tune the Sleepy Pi program (in [sleepy-pi.ino](./sleepy-pi.ino)):

* `POWER_AUTO`: set to `true` to switch the Raspberry Pi on as soon as power is available
  (on the Sleepy Pi)

* `TEMPERATURE_PIN`: set to `14` (A0) or `15` (A1) if you have a `TMP36` or `LM35` temperature
  sensor available; see the [Sleepy Pi Expansion Pinout][sleepy-pi-pinout] for the _A0/A1_ pins
  position

* `POWER_CONTROL`: set to `true` to swith the Raspberry Pi on/off dependending on
  "safe" (low) voltage or (high) temperature thresholds

* `EXPANSION_ENABLE`: set to `true` to power the Sleepy Pi Expansion power along the
  Raspberry Pi (e.g. to control an external fan); see the [Sleepy Pi Expansion Pinout][sleepy-pi-pinout]
  for the _Switched Expansion Power_ pins position

* `EXPANSION_TEMPERATURE`: set to `true` to power the Sleepy Pi Expansion power depending
  on the temperature (requires `TEMPERATURE_PIN` to bet set)

[sleepy-pi-pinout]: https://spellfoundry.com/docs/connecting-the-sleepy-pi-2-expansion-io/


Interacting with the Sleepy Pi
------------------------------

You can interact with the Sleepy Pi from the Raspberry Pi thanks to the [Sleepy Pi client script][sleepy-pi-client]:

``` text
USAGE: sleepy-pi <action> [<options>]

SYNOPSIS:
  Send/retrieve the given data to/from the Sleepy Pi (via the I2C bus).

ACTIONS:

  voltage [read]
    Sleepy Pi supply voltage [V]

  current [read]
    Raspberry Pi drawn current [A]

  temperature [read]
    Ambient temperature [°C]

  shutdown [write]
    Shut down (indefinitely)

  wake-in [write]
    Shut down and wake after the given duration

  wake-at [write]
    Shut down and wake at the given date/time (UTC)

OPTIONS:

  --day, -D
  --hour, -H
  --minute, --min, -M
    Date/time (UTC) or duration
```

[sleepy-pi-client]: ../ansible/roles/sleepypi/files/sleepy-pi.py
