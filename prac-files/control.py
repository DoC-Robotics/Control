# """" Question 6.3 """"


#!/usr/bin/env python
#
# https://www.dexterindustries.com/BrickPi/
# https://github.com/DexterInd/BrickPi3
#
# Copyright (c) 2016 Dexter Industries
# Released under the MIT license (http://choosealicense.com/licenses/mit/).
# For more information, see https://github.com/DexterInd/BrickPi3/blob/master/LICENSE.md
#
# This code is an example for running a motor to a target position set by the encoder of another motor.
# 
# Hardware: Connect EV3 or NXT motors to the BrickPi3 motor ports A and D. Make sure that the BrickPi3 is running on a 9v power supply.
#
# Results:  When you run this program, motor A power will be controlled by the position of motor D. Manually rotate motor B, and motor C's power will change.

from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''

import time     # import the time library for the sleep function
import brickpi3 # import the BrickPi3 drivers

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.

# The following BP.get_motor_encoder function returns the encoder value (what we want to use to control motor C's power).
try:

    left_motor = BP.PORT_A
    right_motor = BP.PORT_D
    power = 50

    BP.set_motor_power(BP.PORT_A, power)
    BP.set_motor_power(BP.PORT_D, power)
    BP.offset_motor_encoder(left_motor, BP.get_motor_encoder(left_motor))
    BP.offset_motor_encoder(right_motor, BP.get_motor_encoder(right_motor))

    BP.set_motor_position(left_motor, 40)
    BP.set_motor_position(right_motor, 40)


    BP.offset_motor_encoder(left_motor, BP.get_motor_encoder(left_motor))
    BP.offset_motor_encoder(right_motor, BP.get_motor_encoder(right_motor))
    BP.set_motor_position(left_motor, -90)
    BP.set_motor_position(right_motor, 90)

    time.sleep(1)
    # rotate 90 degrees
    # BP.set_motor_power(BP.PORT_A, 0)
    # BP.set_motor_power(BP.PORT_D, 20)
    # time.sleep(0.5)
    # BP.set_motor_power(BP.PORT_A, 0)
    # BP.set_motor_power(BP.PORT_D, 0)

except KeyboardInterrupt:
    BP.reset_all()
    break

print(("Motor A Target power: %d" % power), "  Motor A Status: ", BP.get_motor_status(BP.PORT_A))

time.sleep(0.02)  # delay for 0.02 seconds (20ms) to reduce the Raspberry Pi CPU load.

except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.
