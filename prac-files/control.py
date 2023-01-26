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
    power = 15

    BP.set_motor_power(BP.PORT_A, power)
    BP.set_motor_power(BP.PORT_D, power)
    while True:
        try:
            BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A)) # reset encoder A
            BP.offset_motor_encoder(BP.PORT_D, BP.get_motor_encoder(BP.PORT_D)) # reset encoder D
        except IOError as error:
            print(error)
    

        BP.set_motor_position(left_motor, 200)
        BP.set_motor_position(right_motor, 200)


        time.sleep(3)

        try:
            BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A)) # reset encoder A
            BP.offset_motor_encoder(BP.PORT_D, BP.get_motor_encoder(BP.PORT_D)) # reset encoder D
        except IOError as error:
            print(error)
        BP.set_motor_position(left_motor, -180)
        BP.set_motor_position(right_motor, 180)

        time.sleep(3)
    # rotate 90 degrees
    # BP.set_motor_power(BP.PORT_A, 0)
    # BP.set_motor_power(BP.PORT_D, 20)
    # time.sleep(0.5)
    # BP.set_motor_power(BP.PORT_A, 0)
    # BP.set_motor_power(BP.PORT_D, 0)

except KeyboardInterrupt:
    BP.reset_all()

