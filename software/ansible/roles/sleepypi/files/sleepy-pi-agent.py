#!/usr/bin/env python3
# -*- mode:python; tab-width:4; c-basic-offset:4; intent-tabs-mode:nil; -*-
# ex: filetype=python tabstop=4 softtabstop=4 shiftwidth=4 expandtab autoindent smartindent

# REF: https://github.com/SpellFoundry/Sleepy-Pi-Setup/blob/master/shutdowncheck.py

import os
import time

import RPi.GPIO as GPIO


## Constants
SLEEPYPI_AGENT_PIN_SHUTDOWN = 24
SLEEPYPI_AGENT_PIN_STATUS = 25


## Main

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(SLEEPYPI_AGENT_PIN_SHUTDOWN, GPIO.IN)
print(f"Listening from Sleepy Pi shutdown request on GPIO pin {SLEEPYPI_AGENT_PIN_SHUTDOWN}")
GPIO.setup(SLEEPYPI_AGENT_PIN_STATUS, GPIO.OUT)
GPIO.output(SLEEPYPI_AGENT_PIN_STATUS, GPIO.HIGH)
print(f"Telling Sleepy Pi we are running GPIO pin {SLEEPYPI_AGENT_PIN_STATUS}")

# Loop
while True:
    if (GPIO.input(SLEEPYPI_AGENT_PIN_SHUTDOWN)):
        print(f"Sleepy Pi requesting shutdown on GPIO pin {SLEEPYPI_AGENT_PIN_SHUTDOWN}")
        os.system("sudo shutdown -h now")
        break
    time.sleep(0.5)
