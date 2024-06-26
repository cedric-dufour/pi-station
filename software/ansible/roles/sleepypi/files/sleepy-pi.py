#!/usr/bin/env python3
# -*- mode:python; tab-width:4; c-basic-offset:4; intent-tabs-mode:nil; -*-
# ex: filetype=python tabstop=4 softtabstop=4 shiftwidth=4 expandtab autoindent smartindent

import argparse
import time
import sys

import smbus


## Usage/Arguments
oArgumentParser = argparse.ArgumentParser(
    usage="USAGE: %s <action> [<options>]" % sys.argv[0].split("/")[-1],
    description="""
       Send/retrieve the given data to/from the Sleepy Pi (via the I2C bus).
       Date/time must be given in Universal Time Coordinates (UTC).
    """,
)

# Action
oArgumentParser.add_argument(
    "action",
    choices=[
        "version",
        "voltage",
        "current",
        "temperature",
        "watchdog",
        "shutdown",
        "wakein",
        "wakeat",
        "expansionoff",
        "expansionon",
        "watchdogoff",
        "reset",
    ],
)

# Day
oArgumentParser.add_argument(
    "--day",
    "-D",
    type=int,
    default=None,
)

# Hour
oArgumentParser.add_argument(
    "--hour",
    "-H",
    type=int,
    default=None,
)

# Hour
oArgumentParser.add_argument(
    "--minute",
    "--min",
    "-M",
    type=int,
    default=None,
)


## Constants

# I2C
SLEEPYPI_I2C_BUS = 1
SLEEPYPI_I2C_ADDRESS = 0x40
SLEEPYPI_I2C_READ_DELAY = 0.250
SLEEPYPI_I2C_READ_ATTEMPTS = 4
# ... commands
SLEEPYPI_I2C_COMMAND_VOLTAGE_R = 0x01
SLEEPYPI_I2C_COMMAND_CURRENT_R = 0x02
SLEEPYPI_I2C_COMMAND_TEMPERATURE_R = 0x03
SLEEPYPI_I2C_COMMAND_WATCHDOG_R = 0x04
SLEEPYPI_I2C_COMMAND_VERSION_INTERNAL_R = 0x71
SLEEPYPI_I2C_COMMAND_VERSION_USER_R = 0x72
SLEEPYPI_I2C_COMMAND_REQUEST = 0x80
SLEEPYPI_I2C_COMMAND_SHUTDOWN = 0x81
SLEEPYPI_I2C_COMMAND_WAKEIN_W = 0x82
SLEEPYPI_I2C_COMMAND_WAKEAT_W = 0x83
SLEEPYPI_I2C_COMMAND_EXPANSION_OFF = 0x91
SLEEPYPI_I2C_COMMAND_EXPANSION_ON = 0x92
SLEEPYPI_I2C_COMMAND_WATCHDOG_OFF = 0xA2
SLEEPYPI_I2C_COMMAND_RESET = 0xFE


## Helpers


def sleepypi_i2c_write(iCommand, lData=None):
    oI2C.write_i2c_block_data(
        SLEEPYPI_I2C_ADDRESS, iCommand, lData if lData is not None else []
    )


def sleepypi_i2c_read(iCommand):
    mValue = None
    sleepypi_i2c_write(iCommand)
    for _i in range(0, SLEEPYPI_I2C_READ_ATTEMPTS):
        time.sleep(SLEEPYPI_I2C_READ_DELAY)
        try:
            mValue = oI2C.read_word_data(SLEEPYPI_I2C_ADDRESS, SLEEPYPI_I2C_COMMAND_REQUEST)
            if mValue == 0 or mValue == 0xFFFF:  # most likely I2C bus errors
                continue
            break
        except OSError:
            pass
    if mValue is None:
        sys.exit(1)
    return mValue


## Main
oArguments = oArgumentParser.parse_args()
oI2C = smbus.SMBus(SLEEPYPI_I2C_BUS)
if oArguments.action == "version":
    iValue1 = sleepypi_i2c_read(SLEEPYPI_I2C_COMMAND_VERSION_INTERNAL_R)
    iValue2 = sleepypi_i2c_read(SLEEPYPI_I2C_COMMAND_VERSION_USER_R)
    print("{:d}.{:02d} [{:05d}]".format(int(iValue1 / 100), iValue1 % 100, iValue2))
elif oArguments.action == "voltage":
    fValue = sleepypi_i2c_read(SLEEPYPI_I2C_COMMAND_VOLTAGE_R) / 1000
    print(f"{fValue:.3f}")
elif oArguments.action == "current":
    fValue = sleepypi_i2c_read(SLEEPYPI_I2C_COMMAND_CURRENT_R) / 1000
    print(f"{fValue:.3f}")
elif oArguments.action == "temperature":
    fValue = sleepypi_i2c_read(SLEEPYPI_I2C_COMMAND_TEMPERATURE_R) / 100 - 273.16
    print(f"{fValue:.2f}")
elif oArguments.action == "watchdog":
    iValue = sleepypi_i2c_read(SLEEPYPI_I2C_COMMAND_WATCHDOG_R)
    print(f"{iValue:d}")
elif oArguments.action == "shutdown":
    sleepypi_i2c_write(SLEEPYPI_I2C_COMMAND_SHUTDOWN)
elif oArguments.action == "wakein":
    if all(v is None for v in (oArguments.minute, oArguments.hour, oArguments.day)):
        sys.exit(1)
    lData = [
        oArguments.minute or 0,
        oArguments.hour or 0,
        oArguments.day or 0,
    ]
    sleepypi_i2c_write(SLEEPYPI_I2C_COMMAND_WAKEIN_W, lData)
elif oArguments.action == "wakeat":
    if any(v is None for v in (oArguments.minute, oArguments.hour)):
        sys.exit(1)
    lData = [
        oArguments.hour,
        oArguments.minute,
    ]
    if oArguments.day is not None:
        lData.append(oArguments.day)
    sleepypi_i2c_write(SLEEPYPI_I2C_COMMAND_WAKEAT_W, lData)
elif oArguments.action == "expansionoff":
    sleepypi_i2c_write(SLEEPYPI_I2C_COMMAND_EXPANSION_OFF)
elif oArguments.action == "expansionon":
    sleepypi_i2c_write(SLEEPYPI_I2C_COMMAND_EXPANSION_ON)
elif oArguments.action == "watchdogoff":
    sleepypi_i2c_write(SLEEPYPI_I2C_COMMAND_WATCHDOG_OFF)
elif oArguments.action == "reset":
    sleepypi_i2c_write(SLEEPYPI_I2C_COMMAND_RESET)
oI2C.close()
