import math 


def test_arctan():
    '''
    
    Function to just show how arctan works.I'm pretty sure only 2 quadrants need to be mapped and return unaccurate values but this test will show how. 

    '''

    #quadrant 1 test
    x_coord, y_coord = 1,2
    get_q1 = math.atan(y_coord/x_coord)*180/math.pi
    #this is good
    print("Quadrant 1 val:",get_q1)

    #quadrant 2 test -> not so good, add 180 to original value
    x_coord, y_coord = -1,2
    get_q2 = math.atan(y_coord/x_coord)*180/math.pi
    print("Quadrant 2 val:",get_q2)

    #quadrant 3 test -> not so good, subtract 180 from original value
    x_coord, y_coord = -1,-2
    get_q3 = math.atan(y_coord/x_coord)*180/math.pi
    print("Quadrant 3 val:",get_q3)

    #quadrant 4 test -> seems ok and right. 
    x_coord, y_coord = 1,-2
    get_q4 = math.atan(y_coord/x_coord)*180/math.pi
    print("Quadrant 4 val:",get_q4)

    pass


def test_get_angle():

    
    my_two = calc_waypoint(current=[2,3,56.3099324],target=[-3,-2])
    print(my_two)
    #runs some tests for quadrant 1  
    targets_q1 = [[3,5],[5,4],[4,0],[3,0],[-3,-2],[3,-2]]
    currents_q1 = [[1,1,45],[3,2,33.69007],[0,0,0],[2,3,56.3099324],[5,2,0]]

    for i in range(0,len(currents_q1)):
        #deduce angle and observe whether it confirms to specifications. 
        print("Iter:", i," ",calc_waypoint(current=currents_q1[i],target=targets_q1[i]))

    targets = [[0,1],[-1,1],[-2,1]]
    curr_arr = [[-1,-1,-135],[-2,3,123.690],[1,1,-45]]

    for num in range(0,len(curr_arr)):
        tmp,x = calc_waypoint(current=curr_arr[num],target=targets[num])
        print("ITER:",num,"VALS:",tmp,x)
    pass

def arctan_map(x_diff,y_diff,angle_diff):
    '''
    
    Takes the arctanned parameter between x and y we get 
    
    '''
    #covers quadrant 3
    
    if ((x_diff<0) and (y_diff<0)):
        angle_diff -= 180
    
    #covers quadrant 2 and gets value. 
    elif x_diff<0:
        angle_diff += 180
    
    return angle_diff


def get_ang_diff(euc_dist,x_diff,y_diff,target,current):
    #checks the arctan conditions. 
    if (euc_dist == abs(x_diff)):

        if (target[0] >= current[0]):
            print("right X axis")
            angle_diff = 0

        else:
            print("left X axis")
            angle_diff = 180

    elif (euc_dist == abs(y_diff)):
        if (target[1] >= current[1]):
            print("up Y axis")
            angle_diff = 90

        else:
            #needed to keep things between 0 and 180 degrees. 
            print("down Y axis")
            angle_diff = -90

    else:
        angle_diff = math.atan(y_diff/x_diff)*180/math.pi
        angle_diff = arctan_map(x_diff,y_diff,angle_diff)
    
    return angle_diff


# def calc_waypoint(target,current, particles=None):
#     """
#     target [Wx,Wy]
#     current [x,y,theta]
#     """

#     rot_scale = 215.0/90.0 #per 1 degree
#     straight_scale = 640.0/40.0 #per 1 cm
    
#     x_diff = target[0]-current[0]
#     y_diff = target[1]-current[1]

#     euclid_distance = math.sqrt(y_diff**2+x_diff**2)
    
#     #gets an unmapped angle distance. 
#     target_orientation = math.atan2(y_diff, x_diff) * 180 / math.pi
#     rotation_amount = target_orientation - current[2]
#     # print("angle_diff: ", angle_diff)

#     if rotate_amount<-180:
#         rotate_amount += 360

#     elif rotate_amount>180:
#         rotate_amount-=360
    
#     # print("calculating rotation amount:",rotate_amount,"Euclid",euclid_distance)
#     # rotate_and_move(angle_diff=rotate_amount,scale_factor_rot=rot_scale,distance=euclid_distance,sf_straight=straight_scale)
#     # particles.genNewParticlesRotation(rotate_amount)
#     # particles.genNewParticlesStraight(euclid_distance)

#     return rotate_amount,angle_diff



# if __name__=="__main__":

#     #I think the behaviour is right for this case. 
#     # test_get_angle()

#     #uncomment this to see how arctan worked
#     # test_arctan()

#     pass