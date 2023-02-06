# LAB 2 - bump into the wall and recover

# use python 3 syntax but make it compatible with python 2
from __future__ import print_function
from __future__ import division  # ''

import time     # import the time library for the sleep function
import brickpi3  # import the BrickPi3 drivers

# Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
BP = brickpi3.BrickPi3()

try:
    try:
        BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))  # reset encoder B
        BP.offset_motor_encoder(BP.PORT_C, BP.get_motor_encoder(BP.PORT_C))  # reset encoder C
    except IOError as error:
        print(error)

    left_motor = BP.PORT_B
    right_motor = BP.PORT_C
    left_bumper = BP.PORT_3
    right_bumper = BP.PORT_2
    BP.set_sensor_type(left_bumper, BP.SENSOR_TYPE.TOUCH)
    BP.set_sensor_type(right_bumper, BP.SENSOR_TYPE.TOUCH)

    ###### HYPERPARAMETERS ######
    rotate_duration = 3
    reverse_duration = 2
    wait_duration = 2
    straight_speed = 200
    distance_rotation = 220 
    ######                 ######

    BP.set_motor_limits(left_motor, 50, 200)          # set a power limit (in percent) and a speed limit (in Degrees Per Second)
    BP.set_motor_limits(right_motor, 50, 200)

    while True:
        # go straight
        BP.set_motor_dps(left_motor + right_motor, straight_speed)
        # print("after straight:  Motor left Status: ", BP.get_motor_status(left_motor), "Motor right Status: ", BP.get_motor_status(right_motor))

        
        try:
            bumperLEFTState = BP.get_sensor(left_bumper)
            bumperRIGHTState = BP.get_sensor(right_bumper)
            print("left ", bumperLEFTState, " - right ", bumperRIGHTState)
            
            if (bumperLEFTState or bumperRIGHTState):
                
                # time.sleep(wait_duration)
                BP.set_motor_dps(left_motor + right_motor, -straight_speed)
                time.sleep(reverse_duration)
                BP.set_motor_dps(left_motor + right_motor, 0)
                time.sleep(wait_duration)
                
                current_position_left = BP.get_motor_status(left_motor)[2]
                current_position_right = BP.get_motor_status(right_motor)[2]

                if bumperLEFTState:
                    # turn RIGHT 90 degrees (hit left bumper)
                    current_position_left = current_position_left - distance_rotation
                    current_position_right = current_position_right + distance_rotation
                else:
                    # turn LEFT 90 degrees (hit right bumper)
                    current_position_left = current_position_left + distance_rotation
                    current_position_right = current_position_right - distance_rotation

                BP.set_motor_position(left_motor, current_position_left)                
                BP.set_motor_position(right_motor, current_position_right)             
                time.sleep(rotate_duration)   

        except brickpi3.SensorError as error:
            print(error)
        
        # if left bumper pressed, reverse and turn right
        # if right -------------------------------- left
        # if both bumper on/pressed then go backwards


except KeyboardInterrupt:
    BP.reset_all()
