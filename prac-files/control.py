# """" Question 6.3 """"

# use python 3 syntax but make it compatible with python 2
from __future__ import print_function
from __future__ import division  # ''

import time     # import the time library for the sleep function
import brickpi3  # import the BrickPi3 drivers

# Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
BP = brickpi3.BrickPi3()

# The following BP.get_motor_encoder function returns the encoder value (what we want to use to control motor C's power).
try:
    try:
        BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))  # reset encoder A
        BP.offset_motor_encoder(BP.PORT_C, BP.get_motor_encoder(BP.PORT_C))  # reset encoder D
    except IOError as error:
        print(error)

    left_motor = BP.PORT_B
    right_motor = BP.PORT_C

    ###### HYPERPARAMETERS ######
    power_straight = 14  
    pow_dif = 1
    duration_straight = 7 # for 40cm 5 s with power 14
    power_rotation = 9 
    duration_rotation = 5.5 # for 90 degrees 4.5 s with power 8
    stop_duration = 5
    ######                 ######

    ###### HYPERPARAMETERS ######
    distance_straight = 640
    distance_rotation = 220 #230
    current_position_left = 0   #initial position
    current_position_right = 0  #initial position
    ######                 ######

    sides = 4
    squares = 1 #10
    i = 1

    BP.set_motor_limits(left_motor, 50, 200)          # set a power limit (in percent) and a speed limit (in Degrees Per Second)
    BP.set_motor_limits(right_motor, 50, 200)

    while i <= sides*squares:

        print("Iteration: ", i, ", square nb: ", int(i/4), ", square side: ", i%4)

        # go straight on each side
        current_position_left = current_position_left + distance_straight
        current_position_right = current_position_right + distance_straight
        print("left: ", current_position_left, ", right: ", current_position_right)
        # BP.set_motor_position(left_motor + right_motor, current_position_left) # current_position_left = current_position_right
        if i % 2 == 0: 
            BP.set_motor_position(left_motor, current_position_left)
            BP.set_motor_position(right_motor, current_position_right)
        else:
            BP.set_motor_position(right_motor, current_position_right)
            BP.set_motor_position(left_motor, current_position_left)
        time.sleep(stop_duration)
        print("after straight:  Motor left Status: ", BP.get_motor_status(left_motor), "Motor right Status: ", BP.get_motor_status(right_motor))


        # turn 90 degrees towards left
        current_position_left = current_position_left - distance_rotation
        current_position_right = current_position_right + distance_rotation
        print("left: ", current_position_left, ", right: ", current_position_right)

        if i % 2 == 0:
            BP.set_motor_position(left_motor, current_position_left)
            BP.set_motor_position(right_motor, current_position_right)
        else:
            BP.set_motor_position(right_motor, current_position_right)
            BP.set_motor_position(left_motor, current_position_left)
        time.sleep(stop_duration)
        print("after rotation:  Motor left Status: ", BP.get_motor_status(left_motor), "Motor right Status: ", BP.get_motor_status(right_motor))

        i = i+1


except KeyboardInterrupt:
    BP.reset_all()
