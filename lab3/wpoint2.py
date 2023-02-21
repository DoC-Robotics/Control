# import control
import math
import time
import brickpi3
import lab3.particlesMCL as particlesMCL
import test_map
# WORKING WITH particlesMCL

BP = brickpi3.BrickPi3()
NUMBER_OF_PARTICLES = 100

def calc_waypoint(target,current, particles):
    """
    target [Wx,Wy]
    current [x,y,theta]
    """

    rot_scale = 215.0/90.0 #per 1 degree
    straight_scale = 640.0/40.0 #per 1 cm
    
    x_diff = target[0]-current[0]
    y_diff = target[1]-current[1]

    # TODO: Update the coordiantes and angle of particles using particles script. 
    ###JAMES CHECK -> I think I've tested this 12th Feb 9:35PM
    
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
    particles.genNewParticlesRotation(rotate_amount)
    particles.genNewParticlesStraight(euclid_distance)

    return particles

def rotate_and_move(angle_diff,scale_factor_rot,distance,sf_straight):

    '''Processes angles and moves
    
    BP.offset motor encoder(BP.PORT A, BP.get motor encoder(BP.PORT A)):
    resets the encoder count to zero.
    '''
    left_motor = BP.PORT_B
    right_motor = BP.PORT_C
    try:
        BP.offset_motor_encoder(
            BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))  # reset encoder A
        BP.offset_motor_encoder(
            BP.PORT_C, BP.get_motor_encoder(BP.PORT_C))  # reset encoder D
    except IOError as error:
        print(error)

    BP.set_motor_limits(left_motor, 50, 200)
    BP.set_motor_limits(right_motor, 50, 200)
    
    print("In function",sf_straight)
    print("rotation START")
    BP.set_motor_position(left_motor,angle_diff*scale_factor_rot)
    BP.set_motor_position(right_motor,-angle_diff*scale_factor_rot)

    if angle_diff<=90: time.sleep(3)
    else: time.sleep(angle_diff*2.5/90)
    print("rotation END")
    
    try:
        BP.offset_motor_encoder(
            BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))  # reset encoder A
        BP.offset_motor_encoder(
            BP.PORT_C, BP.get_motor_encoder(BP.PORT_C))  # reset encoder D
    except IOError as error:
        print(error)
    print("moving START")
    BP.set_motor_position(left_motor,distance*sf_straight)
    BP.set_motor_position(right_motor,distance*sf_straight)

    if distance<=10: time.sleep(2.5)
    else: time.sleep(distance*2.2/10)
    print("moving END")

    return

def estimated_position_and_orientation(particles):
    est_x, est_y, est_theta = 0,0,0
    for i in range(NUMBER_OF_PARTICLES):
        est_x += particles.coordinates[i][0] * particles.weights[i]
        est_y += particles.coordinates[i][1] * particles.weights[i]
        est_theta += particles.coordinates[i][2] * particles.weights[i]
    
    return est_x, est_y, est_theta
    
if __name__=="__main__":

    # start at (0,0,0)
    particles = particlesMCL.particlesMCL()
    
    # x, y, theta = estimated_position_and_orientation(particles)
    # print("current coords: ",x,y,theta)
    # print("Write down the coordinates you want to get to:")
    # x_goal, y_goal = input("Enter x y: ").split()
    # x_goal, y_goal = float(x_goal),float(y_goal)
    # calc_waypoint([x_goal,y_goal],[x,y,theta], particles)
    current_pos = [0,0,0]
    
    try:
        while True:
            x, y, theta = estimated_position_and_orientation(particles)
            print("current coords: ",x,y,theta)
            print("Write down the coordinates you want to get to:")
            x_goal, y_goal = input("Enter x y: ").split()
            x_goal, y_goal = float(x_goal),float(y_goal)
            new_particles = calc_waypoint([x_goal,y_goal],[x,y,theta], particles)
            particles = new_particles

    except KeyboardInterrupt:
        BP.reset_all()
