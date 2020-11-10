#!/usr/bin/env python3
# -*- mode:python; tab-width:4; c-basic-offset:4; intent-tabs-mode:nil; -*-
# ex: filetype=python tabstop=4 softtabstop=4 shiftwidth=4 expandtab autoindent smartindent

import argparse
import os
import time
import sys

import ephem
import smbus


## Usage/Arguments
oArgumentParser = argparse.ArgumentParser(
    usage="USAGE: %s [<options>]" % sys.argv[0].split("/")[-1],
    description="""
        Shut-down/wake-up the Raspberry Pi at sunset/sunrise.
    """,
)

# Latitude
oArgumentParser.add_argument(
    "--latitude", "-a",
    help="Latitude (degrees)",
    type=float,
    default=float(os.environ.get('SLEEPYPI_SUNWATCH_LATITUDE', 46.95240)),
)

# Longitude
oArgumentParser.add_argument(
    "--longitude", "-o",
    help="Longitude (degrees)",
    type=float,
    default=float(os.environ.get('SLEEPYPI_SUNWATCH_LONGITUDE', 7.439583)),
)

# Shutdown offset
oArgumentParser.add_argument(
    "--shutdown", "-s",
    help="Shut-down time offset (seconds)",
    type=int,
    default=int(os.environ.get('SLEEPYPI_SUNWATCH_SHUTDOWN', 0)),
)

# Wake offset
oArgumentParser.add_argument(
    "--wakeup", "-w",
    help="Wake-up time offset (seconds)",
    type=int,
    default=int(os.environ.get('SLEEPYPI_SUNWATCH_WAKEUP', 0)),
)


## Constants

# I2C
SLEEPYPI_I2C_BUS = 1
SLEEPYPI_I2C_ADDRESS = 0x04
# ... commands
SLEEPYPI_I2C_COMMAND_WAKEAT_W = 0x83


## Main
oArguments = oArgumentParser.parse_args()

# Observer
oObserver = ephem.Observer()
oObserver.lat = str(oArguments.latitude)
oObserver.lon = str(oArguments.longitude)

# Loop
while True:
    oSun = ephem.Sun()
    dNow = ephem.now()

    # Time (next)
    # ... sunset
    oObserver.date = dNow
    dNextSetting = oObserver.next_setting(oSun)
    # ... sunrise
    oObserver.date = dNextSetting
    dNextRising = oObserver.next_rising(oSun)

    # Shutdown
    dShutdown = ephem.date(dNextSetting + oArguments.shutdown * ephem.second)
    print(f"Shutting down at {dShutdown}")
    # ... wait until shutdown time
    fWait = 86400.0 * (dShutdown - dNow)
    if(fWait > 0.0):
        time.sleep(fWait)

    # Wake-up
    dWakeup = ephem.date(dNextRising + oArguments.wakeup * ephem.second)
    print(f"Waking up at {dWakeup}")
    (iYear, iMonth, iDay, iHour, iMinute, iSecond) = dWakeup.tuple()
    # ... send shutdown + wake-up command (I2C)
    oI2C = smbus.SMBus(SLEEPYPI_I2C_BUS)
    oI2C.write_i2c_block_data(SLEEPYPI_I2C_ADDRESS, SLEEPYPI_I2C_COMMAND_WAKEAT_W, [iHour, iMinute, iDay])
