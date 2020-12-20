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
        "voltage",
        "current",
        "temperature",
        "shutdown",
        "wakein",
        "wakeat",
        "expansionoff",
        "expansionon",
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
SLEEPYPI_I2C_ADDRESS = 0x04
SLEEPYPI_I2C_READ_DELAY = 0.250
# ... commands
SLEEPYPI_I2C_COMMAND_VOLTAGE_R = 0x01
SLEEPYPI_I2C_COMMAND_CURRENT_R = 0x02
SLEEPYPI_I2C_COMMAND_TEMPERATURE_R = 0x03
SLEEPYPI_I2C_COMMAND_REFRESH_R = 0x80
SLEEPYPI_I2C_COMMAND_SHUTDOWN_W = 0x81
SLEEPYPI_I2C_COMMAND_WAKEIN_W = 0x82
SLEEPYPI_I2C_COMMAND_WAKEAT_W = 0x83
SLEEPYPI_I2C_COMMAND_EXPANSIONOFF_W = 0x91
SLEEPYPI_I2C_COMMAND_EXPANSIONON_W = 0x92


## Helpers


def sleepypi_i2c_write(iCommand, lData=None):
    oI2C.write_i2c_block_data(
        SLEEPYPI_I2C_ADDRESS, iCommand, lData if lData is not None else []
    )


def sleepypi_i2c_read(iCommand):
    mValue = None
    sleepypi_i2c_write(SLEEPYPI_I2C_COMMAND_REFRESH_R)
    for _i in range(1, 40):
        time.sleep(SLEEPYPI_I2C_READ_DELAY)
        try:
            mValue = oI2C.read_word_data(SLEEPYPI_I2C_ADDRESS, iCommand)
            break
        except OSError:
            pass
    if mValue is None:
        sys.exit(1)
    return mValue


## Main
oArguments = oArgumentParser.parse_args()
oI2C = smbus.SMBus(SLEEPYPI_I2C_BUS)
if oArguments.action == "voltage":
    fValue = sleepypi_i2c_read(SLEEPYPI_I2C_COMMAND_VOLTAGE_R) / 1000
    print(f"{fValue:.3f}")
elif oArguments.action == "current":
    fValue = sleepypi_i2c_read(SLEEPYPI_I2C_COMMAND_CURRENT_R) / 1000
    print(f"{fValue:.3f}")
elif oArguments.action == "temperature":
    fValue = sleepypi_i2c_read(SLEEPYPI_I2C_COMMAND_TEMPERATURE_R) / 100 - 273.16
    print(f"{fValue:.2f}")
elif oArguments.action == "shutdown":
    sleepypi_i2c_write(SLEEPYPI_I2C_COMMAND_SHUTDOWN_W)
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
    sleepypi_i2c_write(SLEEPYPI_I2C_COMMAND_EXPANSIONOFF_W)
elif oArguments.action == "expansionon":
    sleepypi_i2c_write(SLEEPYPI_I2C_COMMAND_EXPANSIONON_W)
