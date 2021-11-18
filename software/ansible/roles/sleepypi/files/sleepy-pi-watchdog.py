#!/usr/bin/env python3
# -*- mode:python; tab-width:4; c-basic-offset:4; intent-tabs-mode:nil; -*-
# ex: filetype=python tabstop=4 softtabstop=4 shiftwidth=4 expandtab autoindent smartindent

import argparse
import time
import os
import sys

import smbus


## Usage/Arguments
oArgumentParser = argparse.ArgumentParser(
    usage="USAGE: %s [<options>]" % sys.argv[0].split("/")[-1],
    description="""
        Send regular heartbeat to the Raspberry Pi watchdog.
    """,
)

# Interval
oArgumentParser.add_argument(
    "--interval", "-i",
    help="Heartbeat interval (seconds)",
    type=int,
    default=int(os.environ.get('SLEEPYPI_WATCHDOG_INTERVAL', 60)),
)

# Verbose
oArgumentParser.add_argument(
    "--verbose",
    help="Verbose output",
    action="store_true",
    default=False,
)


## Constants

# I2C
SLEEPYPI_I2C_BUS = 1
SLEEPYPI_I2C_ADDRESS = 0x40
# ... commands
SLEEPYPI_I2C_COMMAND_WATCHDOG_PING = 0xA1


## Main
oArguments = oArgumentParser.parse_args()
iInterval = oArguments.interval
bVerbose = oArguments.verbose

# Loop
while True:
    # Heartbeat
    if(bVerbose):
        print(f"Heartbeat")
    # ... send heartbeat command (I2C)
    oI2C = smbus.SMBus(SLEEPYPI_I2C_BUS)
    oI2C.write_i2c_block_data(SLEEPYPI_I2C_ADDRESS, SLEEPYPI_I2C_COMMAND_WATCHDOG_PING, [])
    oI2C.close()
    time.sleep(iInterval)
