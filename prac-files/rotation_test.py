# use python 3 syntax but make it compatible with python 2
from __future__ import print_function
from __future__ import division

import brickpi3  # import the BrickPi3 drivers
import time

BP = brickpi3.BrickPi3()
# rot_param = [5, 10, 15, 20, 25]  # isolate where which power range is right
power_rotation = 8  # isolate where which power range is right
duration_rotation = [4.8, 4.9, 5]
left_motor = BP.PORT_B
right_motor = BP.PORT_C

BP = brickpi3.BrickPi3()
try:
    try:
        BP.offset_motor_encoder(
            left_motor, BP.get_motor_encoder(left_motor))  # reset encoder A
        BP.offset_motor_encoder(
            right_motor, BP.get_motor_encoder(right_motor))  # reset encoder D
    except IOError as error:
        print(error)

    mode = 0

    for entry in duration_rotation:
        print("current rotation speed is: ", power_rotation, " with duration: ", entry)
        BP.set_motor_power(left_motor, power_rotation)
        BP.set_motor_power(right_motor, -power_rotation)
        time.sleep(entry)
        BP.set_motor_power(left_motor + right_motor, 0)
        time.sleep(7)

except KeyboardInterrupt:
    BP.reset_all()
    print("failed to run motor")
