# import control
import math
import time
import brickpi3
import particles

BP = brickpi3.BrickPi3()
NUMBER_OF_PARTICLES = 100

def calc_waypoint(target,current, particles):
    """
    target [Wx,Wy]
    current [x,y,theta]
    """

    rot_scale = 220/90 #as 220 needed to rotate 90. 
    
    x_diff = target[0]-current[0]
    y_diff = target[1]-current[1]

    # TODO: Update the coordiantes and angle of particles using particles script. 
    euclid_distance = math.sqrt(y_diff**2+x_diff**2)

    angle_diff = math.atan(y_diff/x_diff)*180/math.pi

    #track angle needed to rotate by
    rotate_amount = angle_diff-current[2]
    print("calculating rotation amount:",rotate_amount,"Euclid",euclid_distance)
    rotate_and_move(angle_diff=rotate_amount,scale_factor_rot=rot_scale,distance=euclid_distance,sf_straight=640/40)


def rotate_and_move(angle_diff,scale_factor_rot,distance,sf_straight):

    '''Processes angles and moves
    
    BP.offset motor encoder(BP.PORT A, BP.get motor encoder(BP.PORT A)):
    resets the encoder count to zero.
    '''
    left_motor = BP.PORT_B
    right_motor = BP.PORT_C
    
    BP.offset_motor_encoder(left_motor, BP.get_motor_encoder(left_motor))
    BP.offset_motor_encoder(right_motor, BP.get_motor_encoder(right_motor))
    
    print("In function",sf_straight)

    BP.set_motor_position(left_motor,angle_diff*scale_factor_rot)
    BP.set_motor_position(right_motor,angle_diff*scale_factor_rot)
    
    print("rotation complete")
    time.sleep(1)
    #we 
    BP.offset_motor_encoder(left_motor, BP.get_motor_encoder(left_motor))
    BP.offset_motor_encoder(right_motor, BP.get_motor_encoder(right_motor))
    print("moving")

    BP.set_motor_position(left_motor,distance*640/160)
    BP.set_motor_position(right_motor,distance*640/160)

    time.sleep(1)

    return
    # angle_diff = 
    #math.arctan(1)
    
#220 is the distance to rotation 

def estimated_position_and_orientation(particles):
    est_x,est_y,est_theta = 0
    for i in range(NUMBER_OF_PARTICLES):
        x += particles.coordinates[i][0] * particles.weights[i]
        y += particles.coordinates[i][1] * particles.weights[i]
        theta += particles.coordinates[i][2] * particles.weights[i]
    
    return est_x, est_y, est_theta
    
if __name__=="__main__":
    # current = control.some_function()
    # print(math.arctan(1))
    particles = particles.particles()
    x, y, theta = estimated_position_and_orientation(particles)
    print("current coords: ",x,y,theta)
    print("Write down the coordinates you want to get to:")
    x_goal, y_goal = input("Enter x y: ").split()
    calc_waypoint([x_goal,y_goal],[x,y,theta])
    
