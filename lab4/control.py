# """" Question 6.3 """"

# use python 3 syntax but make it compatible with python 2
from __future__ import print_function
from __future__ import division  # ''
import particles

import time     # import the time library for the sleep function
import brickpi3  # import the BrickPi3 drivers
import math
import statistics

# Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
BP = brickpi3.BrickPi3()

# The following BP.get_motor_encoder function returns the encoder value (what we want to use to control motor C's power).
try:
    try:
        BP.offset_motor_encoder(
            BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))  # reset encoder A
        BP.offset_motor_encoder(
            BP.PORT_C, BP.get_motor_encoder(BP.PORT_C))  # reset encoder D
    except IOError as error:
        print(error)

    left_motor = BP.PORT_B
    right_motor = BP.PORT_C
    ultrasonic_sensor = BP.PORT_1
    BP.set_sensor_type(ultrasonic_sensor, BP.SENSOR_TYPE.NXT_ULTRASONIC)

    ###### HYPERPARAMETERS ######
    rotation_duration = 4
    straight_duration = 2.5
    distance_straight = 160 # per 10 cm
    distance_rotation = 215  # 220
    # current_position_left = 0  # initial position
    # current_position_right = 0  # initial position
    distance_graphics = 155
    rotation_graphics = -math.pi/2
    target_distance = 20   #cm 
    ######                 ######

    sides = 4
    squares = 1  # 10
    j = 1
    readings = []

    # set a power limit (in percent) and a speed limit (in Degrees Per Second)
    BP.set_motor_limits(left_motor, 50, 200)
    BP.set_motor_limits(right_motor, 50, 200)
    particles = particles.particles()

    while True:
        #   reset motor states
        try:
            BP.offset_motor_encoder( left_motor, BP.get_motor_encoder(left_motor))  
            BP.offset_motor_encoder( right_motor, BP.get_motor_encoder(right_motor))
        except IOError as error:
            print(error)

        # take measurement of sonar
        try:
            ultrasonicState = BP.get_sensor(ultrasonic_sensor)
        except brickpi3.SensorError as error:
            print(error)

        print("ultrasonic reading: ", ultrasonicState )

        if len(readings) < 5:
            readings.append(ultrasonicState)
        else:
            readings.pop(0)
            readings.append(ultrasonicState)
        # print("ultrasonit reading: ", statistics.median(readings))
        median_reading = statistics.median(readings)

        if median_reading <= target_distance:
            # if something detected in front of rover
            # rotate 90 to left
            BP.set_motor_position(left_motor, distance_rotation)
            BP.set_motor_position(right_motor, -distance_rotation)
            ##### call prediction for one step
            particles.genNewParticlesRotation(rotation_graphics)
            time.sleep(rotation_duration)
            print("after rotation:  Motor left Status: ", BP.get_motor_status(
                left_motor), "Motor right Status: ", BP.get_motor_status(right_motor))

        else:  
            # if clear way in front of rover      
            # go straight for 10cm
            BP.set_motor_position(left_motor + right_motor, distance_straight)                
            ##### call prediction for one step
            particles.genNewParticlesStraight(distance_graphics)
            time.sleep(straight_duration)

            print("after straight:  Motor left Status: ", BP.get_motor_status(
                left_motor), "Motor right Status: ", BP.get_motor_status(right_motor))


except KeyboardInterrupt:
    BP.reset_all()
