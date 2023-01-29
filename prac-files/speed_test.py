# use python 3 syntax but make it compatible with python 2
from __future__ import print_function
from __future__ import division  # ''


import brickpi3  # import the BrickPi3 drivers
import time

# Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
BP = brickpi3.BrickPi3()
# range of speeds to test
speeds_power = [10, 11, 12, 13, 14]
try:
    try:
        BP.offset_motor_encoder(
            BP.PORT_A, BP.get_motor_encoder(BP.PORT_A))  # reset encoder A
        BP.offset_motor_encoder(
            BP.PORT_D, BP.get_motor_encoder(BP.PORT_D))  # reset encoder D
    except IOError as error:
        print(error)

    for speed_pow in speeds_power:
        print("current speed is: ", speed_pow)
        BP.set_motor_power(BP.PORT_A + BP.PORT_D, speed_pow)
        # BP.set_motor_power(BP.PORT_D, speed)
        time.sleep(5)
        BP.set_motor_power(BP.PORT_A + BP.PORT_D, 0)
        # BP.set_motor_power(BP.PORT_D, 0)
        time.sleep(12)

except KeyboardInterrupt:
    BP.reset_all()
    print("failed to run motor")
