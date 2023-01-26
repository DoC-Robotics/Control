from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division     

import brickpi3 # import the BrickPi3 drivers
import time

rot_param = [5,10,15,20,25]#isolate where which power range is right
BP = brickpi3.BrickPi3() 

for entry in rot_param:
    BP.set_motor_power(BP.PORT_A,entry)
    BP.set_motor_power(BP.PORT_D,-entry)
    time.sleep(6)
    BP.set_motor_power(BP.PORT_A,0)
    BP.set_motor_power(BP.PORT_A,0)
    time.sleep(3)






