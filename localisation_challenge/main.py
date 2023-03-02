# import control
import math
import time
import brickpi3
import test_map
import likelihood
import normalising_resampling
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
rot_scale = 220.0/90.0 #per 1 degree #218
straight_scale = 640.0/40.0 #per 1 cm
map = likelihood.initialise_map()


def calc_waypoint(target,current):
    """
    target [Wx,Wy]
    current [x,y,theta]
    """
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
    
    # particles.genNewParticlesRotation(rotate_amount)
    # particles.genNewParticlesStraight(euclid_distance)
    print("----Exit Waypoint-----: ")
    return rotate_amount, rot_scale, euclid_distance, straight_scale


def move_rover(rotate_amount,  euclid_distance, particles):
    left_motor = BP.PORT_B
    right_motor = BP.PORT_C
    BP.set_motor_limits(left_motor, 50, 200)
    BP.set_motor_limits(right_motor, 50, 200)
    rotate_rover(rotate_amount,particles)
    move_rover_forward(euclid_distance, particles) 
    return particles


def rotate_rover(rotate_amount, particles):
    try:
        BP.offset_motor_encoder( left_motor, BP.get_motor_encoder(left_motor)) 
        BP.offset_motor_encoder( right_motor, BP.get_motor_encoder(right_motor)) 
    except IOError as error:
        print(error)
    print("rotation START")
    BP.set_motor_position(left_motor,rotate_amount*rot_scale)
    BP.set_motor_position(right_motor,-rotate_amount*rot_scale)
    particles.genNewParticlesRotation(rotate_amount)
    print("rotation END")

def move_rover_forward(euclid_distance, particles):
    try:
        BP.offset_motor_encoder( left_motor, BP.get_motor_encoder(left_motor)) 
        BP.offset_motor_encoder( right_motor, BP.get_motor_encoder(right_motor)) 
    except IOError as error:
        print(error)

    dist  = euclid_distance
    print("Distance to Target: ", euclid_distance)
    if euclid_distance> 20:
        BP.set_motor_position(left_motor + right_motor, 20*straight_scale)
        dist = 20
        print("Moving 20")
    else: 
        BP.set_motor_position(left_motor + right_motor, euclid_distance*straight_scale)
        print("Moving ", euclid_distance)
    particles.genNewParticlesStraight(dist)

def move_rover_to_target(particles, x_goal, y_goal):
    
    x, y, theta = estimated_position_and_orientation(particles)
    print("X, Y DIFF :" ,abs(x_goal-x), abs(y_goal -y))
    rotate_amount, rot_scale, euclid_distance, straight_scale = calc_waypoint([x_goal, y_goal], [x,y,theta])
    particles = move_rover(rotate_amount, rot_scale, euclid_distance, straight_scale, particles)
    particles = normalising_resampling.normalising_and_resampling(particles)
    
    return particles

def distance_measured():
    length_rover = 7.5
    # target_distance = 20 #cm
    ultrasonic_sensor = BP.PORT_2
    BP.set_sensor_type(ultrasonic_sensor, BP.SENSOR_TYPE.NXT_ULTRASONIC)

    #   take measurement of sonar for 2 seconds and take median
    try:
        ultrasonicState = BP.get_sensor(ultrasonic_sensor)
        reading = ultrasonicState + length_rover
    except brickpi3.SensorError as error:
        print(error)
        time.sleep(0.1)
        BP.reset_all()
        ultrasonic_sensor = BP.PORT_2
        BP.set_sensor_type(ultrasonic_sensor, BP.SENSOR_TYPE.NXT_ULTRASONIC)

    return reading



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
       

    mean_angle = math.atan2(sum_sin, sum_cos)
    est_theta = mean_angle * 180 / math.pi
    return est_x[0], est_y[0], est_theta

def find_initial_position(particles):
    rotate_amount = 360
    rotate_rover(rotate_amount, particles)
        while (BP.get_motor_endcoder(left_motor) != rotate_amount and BP.get_motor_endcoder(right_motor)!= -rotate_amount):
            particles = normalising_resampling.normalising_and_resampling(particles)
    
if __name__=="__main__":

    map.draw()
    start_flag = True
    path=[(84,30),(180,30),(180,54),(138,54),(138,168)]    # path=[(180,54),(138,54),(138,168),(114,168),(114,84),(84,84),(84,30)]
    particles = particlesMCL.particlesMCL()

    intialise_index = find_initial_position(particles)
    path_index = intialise_index
    ###### LOOPING THROUGH THE GIVEN PATH
    while(True):
        try:
            x, y, theta = estimated_position_and_orientation(particles)
            x_goal, y_goal = path[path_index%4][0], path[path_index%4][1] 
            print("X_goal, y_goal", x_goal,y_goal)
            if (abs(x_goal-x)<5 and abs(y_goal-y)<5):
                path_index+=1
                time.sleep(1)
                
            # update particle weghts    
            distance_measurement = distance_measured()
            likelihood.update_particles_weights(particles, distance_measurement, map)
            
            # move rober if angle paths deviates too much
            particles = move_rover_to_target(particles,x_goal,y_goal)
             
            particles.printParticles(particles.convertNPtoTuples(particles))
            particles.printPaths(particles, map)
            

        except KeyboardInterrupt:
            BP.reset_all()

            