from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''


import brickpi3 # import the BrickPi3 drivers
import time

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
# range of speeds to test
speeds = [10, 11, 12, 13, 14]
try:
    for speed in speeds:
        print("current speed is: ", speed)
        BP.set_motor_power(BP.PORT_A,speed)
        BP.set_motor_power(BP.PORT_D,speed)
        time.sleep(5)
        BP.set_motor_power(BP.PORT_A,0)
        BP.set_motor_power(BP.PORT_D,0)
        time.sleep(12)

except KeyboardInterrupt:
    BP.reset_all()
    print("failed to run motor")
    




