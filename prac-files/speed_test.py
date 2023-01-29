# use python 3 syntax but make it compatible with python 2
from __future__ import print_function
from __future__ import division  # ''


import brickpi3  # import the BrickPi3 drivers
import time

# Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
BP = brickpi3.BrickPi3()
# range of speeds to test
speeds_power = [0,0.1,0.2,0.3,0.5]
speed_pow = 14
left_motor = BP.PORT_B
right_motor = BP.PORT_C

try:
    try:
        BP.offset_motor_encoder(
            left_motor, BP.get_motor_encoder(left_motor))  # reset encoder A
        BP.offset_motor_encoder(
            right_motor, BP.get_motor_encoder(right_motor))  # reset encoder D
    except IOError as error:
        print(error)

    for pow_dif in speeds_power:
        print("current speed is: ", speed_pow, ", with pow diff: ", pow_dif)
        BP.set_motor_power(left_motor + right_motor, speed_pow)
        time.sleep(0.5)
        BP.set_motor_power(right_motor, speed_pow-pow_dif)
        time.sleep(3)
        BP.set_motor_power(left_motor + right_motor, 0)
        time.sleep(10)

except KeyboardInterrupt:
    BP.reset_all()
    print("failed to run motor")
