from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #                           ''

import time     # import the time library for the sleep function
import brickpi3 # import the BrickPi3 drivers

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.

try:
    try:
        BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A)) # reset encoder A
        BP.offset_motor_encoder(BP.PORT_D, BP.get_motor_encoder(BP.PORT_D)) # reset encoder D
    except IOError as error:
        print(error)
    
    BP.set_motor_power(BP.PORT_D, BP.MOTOR_FLOAT)    # float motor D
    BP.set_motor_limits(BP.PORT_A, 50, 200)          # optionally set a power limit (in percent) and a speed limit (in Degrees Per Second)
    while True:
        # Each of the following BP.get_motor_encoder functions returns the encoder value.
        try:
            target = BP.get_motor_encoder(BP.PORT_D) # read motor D's position
        except IOError as error:
            print(error)
        
        BP.set_motor_position(BP.PORT_A, target)    # set motor A's target position to the current position of motor D
        
        try:
            print("Motor A target: %6d  Motor A position: %6d" % (target, BP.get_motor_encoder(BP.PORT_A)))

            get_vals = BP.get_motor_status(BP.PORT_A)
            with open('op_file.txt','a') as op_file:
                op_file.write(str(get_vals[0])+","get_vals[1])

        except IOError as error:
            print(error)
        
        time.sleep(0.02)  # delay for 0.02 seconds (20ms) to reduce the Raspberry Pi CPU load.

except KeyboardInterrupt: # except the program gets interrupted by Ctrl+C on the keyboard.
    BP.reset_all()        # Unconfigure the sensors, disable the motors, and restore the LED to the control of the BrickPi3 firmware.
