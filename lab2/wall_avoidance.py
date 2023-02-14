# LAB 2 - aim to keep the rover at a targeted distance of 30cm away from the obstacle(wall)

# use python 3 syntax but make it compatible with python 2
from __future__ import print_function
from __future__ import division  # ''

import time     # import the time library for the sleep function
import brickpi3  # import the BrickPi3 drivers
import statistics

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
    ultrasonic_sensor = BP.PORT_1

    BP.set_sensor_type(left_bumper, BP.SENSOR_TYPE.TOUCH)
    BP.set_sensor_type(right_bumper, BP.SENSOR_TYPE.TOUCH)
    BP.set_sensor_type(ultrasonic_sensor, BP.SENSOR_TYPE.NXT_ULTRASONIC)

    ###### HYPERPARAMETERS ######
    rotate_duration = 3
    reverse_duration = 2
    wait_duration = 2
    max_speed = 100 # will gradually slow down when approaching wall
    distance_rotation = 220 
    proportion_control_const = 5
    target_distance = 10   #cm -> always aim to keep the rover at a targeted distance of 30cm away from the obstacle(wall)
    ######                 ######

    BP.set_motor_limits(left_motor, 50, 50)          # set a power limit (in percent) and a speed limit (in Degrees Per Second)
    BP.set_motor_limits(right_motor, 50, 50)

    readings = []

    while True:
        # go straight
        # print("after straight:  Motor left Status: ", BP.get_motor_status(left_motor), "Motor right Status: ", BP.get_motor_status(right_motor))
        
        
        try:
            ultrasonicState = BP.get_sensor(ultrasonic_sensor)
            bumperLEFTState = BP.get_sensor(left_bumper)
            bumperRIGHTState = BP.get_sensor(right_bumper)
            print("ultrasonic reading: ", ultrasonicState ," - bumpers left ", bumperLEFTState, " - right ", bumperRIGHTState)
            
            if len(readings) < 5:
                readings.append(ultrasonicState)
            else:
                readings.pop(0)
                readings.append(ultrasonicState)
            # print("ultrasonit reading: ", statistics.median(readings))
            difference = statistics.median(readings) - target_distance
            BP.set_motor_dps(left_motor + right_motor, proportion_control_const * difference)
            # time.sleep(1)
            
            # ultrasonic target
            # take difference between current distance and targeted dist (30cm)
            # set velocity to (dist diff)*K, K being a proportional constant
            

            # bumper stop
            if (bumperLEFTState or bumperRIGHTState):
                
                time.sleep(wait_duration)
                BP.set_motor_dps(left_motor + right_motor, -max_speed)
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
