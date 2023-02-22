# import control
import math
import time
import brickpi3
import test_map
import likelihood
import normalising_resampling3_2
import particlesMCL
import statistics
import math

BP = brickpi3.BrickPi3()
BP.reset_all()
left_motor = BP.PORT_B
right_motor = BP.PORT_C
ultrasonic_sensor = BP.PORT_2
BP.set_sensor_type(ultrasonic_sensor, BP.SENSOR_TYPE.NXT_ULTRASONIC)

NUMBER_OF_PARTICLES = 100
map = likelihood.initialise_map()
particles = particlesMCL.particlesMCL()


def calc_waypoint(target,current, particles):
    """
    target [Wx,Wy]
    current [x,y,theta]
    """

    rot_scale = 220.0/90.0 #per 1 degree #218
    straight_scale = 640.0/40.0 #per 1 cm
    
    x_diff = target[0]-current[0]
    y_diff = target[1]-current[1]
    print("----In Waypoint---- x_diff: ", x_diff)
    print("y_diff: ", y_diff)
    
    euclid_distance = math.sqrt(y_diff**2+x_diff**2)
    angle_diff = test_map.get_ang_diff(euclid_distance,x_diff,y_diff,target,current)

    rotate_amount = angle_diff-current[2]
    print("calculating rotation amount:",rotate_amount,"Euclid",euclid_distance)

    #swing to adjust between pi and -pi. 
    if rotate_amount<-180:
        rotate_amount += 360

    elif rotate_amount>180:
        rotate_amount-=360

    rotate_and_move(angle_diff=rotate_amount,scale_factor_rot=rot_scale,distance=euclid_distance,sf_straight=straight_scale)
    
    # particles.genNewParticlesRotation(rotate_amount)
    # particles.genNewParticlesStraight(euclid_distance)
    print("----Exit Waypoint-----: ")

    return particles

def rotate_and_move(angle_diff,scale_factor_rot,distance,sf_straight):

    '''Processes angles and moves
    
    BP.offset motor encoder(BP.PORT A, BP.get motor encoder(BP.PORT A)):
    resets the encoder count to zero.
    '''
    left_motor = BP.PORT_B
    right_motor = BP.PORT_C
    BP.set_motor_limits(left_motor, 50, 200)
    BP.set_motor_limits(right_motor, 50, 200)

    #   reset motor states
    try:
        BP.offset_motor_encoder( left_motor, BP.get_motor_encoder(left_motor)) 
        BP.offset_motor_encoder( right_motor, BP.get_motor_encoder(right_motor)) 
    except IOError as error:
        print(error)

    print("In function - robot movement")
    print("rotation START")
    BP.set_motor_position(left_motor,angle_diff*scale_factor_rot)
    BP.set_motor_position(right_motor,-angle_diff*scale_factor_rot)
    particles.genNewParticlesRotation(angle_diff)
    if angle_diff<=90: time.sleep(3)
    else: time.sleep(angle_diff*2.5/90)
    print("rotation END")
    
    #   reset motor states
    try:
        BP.offset_motor_encoder( left_motor, BP.get_motor_encoder(left_motor)) 
        BP.offset_motor_encoder( right_motor, BP.get_motor_encoder(right_motor)) 
    except IOError as error:
        print(error)

    # MOVING IN STEPS OF 20CM 
    print("moving START")
    steps_dist = 20
    num_twenty = math.floor(distance/20)
    dist_remainder = distance%20
    for i in range(1,num_twenty+1):
        dist = i*steps_dist
        print("steps dist:", dist)
        BP.set_motor_position(left_motor + right_motor,dist*sf_straight)
        # BP.set_motor_position(right_motor,dist*sf_straight)
        while BP.get_motor_status(left_motor)[2] < math.floor(dist*sf_straight):
            # print(BP.get_motor_status(left_motor), dist*sf_straight)
            time.sleep(1)
        particles.genNewParticlesStraight(steps_dist)
        time.sleep(0.5)
    print("moving END")
    
    print("remainder dist: ", dist_remainder, " - total dist move: ", steps_dist*num_twenty+dist_remainder)
    BP.set_motor_position(left_motor + right_motor,(steps_dist*num_twenty+dist_remainder)*sf_straight)
    # BP.set_motor_position(right_motor,(steps_dist*num_twenty+dist_remainder)*sf_straight)
    particles.genNewParticlesStraight(dist_remainder)
    time.sleep(1)

    return

def distance_measured():
    readings = []
    length_rover = 7.5
    # target_distance = 20 #cm
    ultrasonic_sensor = BP.PORT_2
    BP.set_sensor_type(ultrasonic_sensor, BP.SENSOR_TYPE.NXT_ULTRASONIC)

    #   take measurement of sonar for 2 seconds and take median
    t_end = time.time() + 6 # 2 seconds
    while time.time() < t_end:
        try:
            ultrasonicState = BP.get_sensor(ultrasonic_sensor)
            readings.append(ultrasonicState + length_rover)
            print("ultrasonic reading at time ",time.time()," : ", ultrasonicState + length_rover)
        except brickpi3.SensorError as error:
            print(error)
            time.sleep(0.25)
            BP.reset_all()
            ultrasonic_sensor = BP.PORT_2
            BP.set_sensor_type(ultrasonic_sensor, BP.SENSOR_TYPE.NXT_ULTRASONIC)

        time.sleep(0.2)

    if len(readings) == 0:
        return distance_measured()
    else:
        median_reading = statistics.median(readings)
        print("Real reading to the facing wall: ", median_reading)
        # NO OBSTACLE AVOIDANCE, AS NOT NEEDED IN THE GIVEN PATH
        # median_reading only used for the updating of MCL particles
        return median_reading




def estimated_position_and_orientation(particles):
    est_x, est_y, est_theta = 0,0,0
    
    # circular mean operation to deal with wrapping of coordinates
    sum_cos = 0
    sum_sin = 0

    for i in range(NUMBER_OF_PARTICLES):
        est_x += particles.coordinates[i][0] * particles.weights[i]
        est_y += particles.coordinates[i][1] * particles.weights[i]
        
        sum_cos =  math.cos(particles.coordinates[i][2]* math.pi / 180) * particles.weights[i]
        sum_sin =  math.sin(particles.coordinates[i][2]* math.pi / 180) * particles.weights[i]
        print("Weight ",particles.weights[i], " for theta ", particles.coordinates[i][2] )

    mean_angle = math.atan2(sum_sin, sum_cos)
    est_theta = mean_angle * 180 / math.pi
    return est_x, est_y, est_theta

if __name__=="__main__":

    map.draw()
    start_flag = True
    path=[(84,30),(180,30),(180,54),(138,54),(138,168),(114,168),(114,84),(84,84),(84,30)]
    # path=[(180,54),(138,54),(138,168),(114,168),(114,84),(84,84),(84,30)]
    ###### LOOPING THROUGH THE GIVEN PATH
    try:
        for path_index in range(0, len(path)):
            if path_index == 0:
                print("Coordinates you are at (are starting from):", path[0])
                start_flag = False
                particles.initialise_at(path[0][0],path[0][1])
                # particles.new_initialization(path[0][0],path[0][1], 90)
            else:
                print("(Example before weight estimation) coordinates[0]: ", particles.coordinates[0][0], particles.coordinates[0][1], particles.coordinates[0][2])
                x, y, theta = estimated_position_and_orientation(particles)
                print("current coords: ",x,y,theta)
                print("Coordinates you want to get to:", path[path_index])
                x_goal, y_goal = path[path_index][0], path[path_index][1] 
                print("x_goal: ", x_goal)
                print("y_goal: ", y_goal)

                # Moves robot to the position given and speads the particles
                spread_particles = calc_waypoint([x_goal,y_goal],[x,y,theta], particles) 
                
                # Update weights based on likelihood function
                distance_measurement = distance_measured() # measures the distance with the sonar sensor
                likelihood.update_particles_weights(spread_particles, distance_measurement, map)
                
                # Resampling a new set of particles
                particles = normalising_resampling3_2.normalising_and_resampling(spread_particles)
                print("---- Resampled particles printed")
                #print(particles.convertNPtoTuples(particles))
                particles.printParticles(particles.convertNPtoTuples(particles))
            time.sleep(3)

    ######### WITH USER INPUT    
    # try:
    #     while True:
    #         if start_flag:
    #             print("Write down the coordinates you are at (are starting from):")
    #             x_start, y_start = input("Enter x y: ").split()
    #             start_flag = False
    #             particles.initialise_at(x_start, y_start)

    #         x, y, theta = estimated_position_and_orientation(particles)
    #         print("current coords: ",x,y,theta)
    #         print("Write down the coordinates you want to get to:")
    #         x_goal, y_goal = input("Enter x y: ").split()
    #         x_goal, y_goal = float(x_goal),float(y_goal)

    #         # Moves robot to the position given and speads the particles
    #         spread_particles = calc_waypoint([x_goal,y_goal],[x,y,theta], particles) 
            
    #         # Update weights based on likelihood function
    #         distance_measurement = distance_measured() # measures the distance with the sonar sensor
    #         likelihood.update_particles_weights(spread_particles, distance_measurement, map)
            
    #         # Resampling a new set of particles
    #         particles = normalising_resampling3_2.normalising_and_resampling(spread_particles)

            
    except KeyboardInterrupt:
        BP.reset_all()

            