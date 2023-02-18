import math
import mapParticleClasses
import particlesMCL

import numpy as np

# Likelihood for lab3.1 without normalisation and using the random particles given by the professor (mapParticleClasses), 
# weights not updated in our own particles class

# TODO: 
# DONE - loop through all walls and find the wall with a coordinate that is on the line of the particle with orientation theta 
# DONE - for x_coord_wall we it needs to be equal to single_particle[0]+(Z*math.cos(single_particle[2])
# DONE - for y_coord_wall we it needs to be equal to single_particle[1]+(Z*math.cos(single_particle[2])
# DONE - where Z is just a random range that would reach all walls on the map
# DONE - if line intersects then that is the wall that can be reached, and then we calculate the distance m to it
# marco done - then look at the difference between m and the actual measurement z and calculate a likelihood value using a Gaussian model


def calculate_distance_to_wall(x_coor, y_coor, theta, wall_a_x_coor,wall_a_y_coor, wall_b_x_coor, wall_b_y_coor):
    '''

    m ={(By - Ay )(Ax - x) - (Bx - Ax )(Ay - y)}/{(By - Ay) cos θ - (Bx - Ax) sin θ}
    Gets distance to wall based on x,y, theta parameters.  

    '''
    if ((theta == 90 or theta == 270) and wall_b_x_coor-wall_a_x_coor == 0):
        return float('inf')
    elif ((theta == 0 or theta == 180 or theta == 360) and wall_b_y_coor-wall_a_y_coor == 0):
        return float('inf')
    # division by 0 if theta is 90 and wall y is on x axis (0 coords), same problem if 180 theta with wall on y axis (0 coords)
    # parallel case when walls starting from origin
    # (0 - 0)*1 - (168-0)*0 = 0 -> particle with orientation +-90degreees, and wall OA
    # (210-0)*0 - (0 - 0)*1 -> particle with orientation +-180degrees, and wall OH
    

    #gives the numerator and denominator for the distance to the wall. 
    numerator = (wall_b_y_coor-wall_a_y_coor) * (wall_a_x_coor - x_coor) - (wall_b_x_coor - wall_a_x_coor) * (wall_a_y_coor - y_coor)
    denominator = (wall_b_y_coor - wall_a_y_coor)*math.cos(theta) - (wall_b_x_coor - wall_a_x_coor) * math.sin(theta)
    distance = (numerator) / (denominator)
    
    return distance
        

def initialise_map():
    mymap = mapParticleClasses.Map();
    # Definitions of walls
    '''
    
    a: O to A, b: A to B, c: C to D, d: D to E
    e: E to F, f: F to G, g: G to H ,h: H to O
    
    '''
    mymap.add_wall((0,0,0,168));        # a (wall OA)
    mymap.add_wall((0,168,84,168));     # b (wall OH)
    mymap.add_wall((84,126,84,210));    # c  (CD)
    mymap.add_wall((84,210,168,210));   # d (BD)
    mymap.add_wall((168,210,168,84));   # e -> we use a particle and will calculate the lowest distance. 
    mymap.add_wall((168,84,210,84));    # f
    mymap.add_wall((210,84,210,0));     # g
    mymap.add_wall((210,0,0,0));        # h
    return mymap

def initialise_particles():
    #initializes particles and returns these. 
    particles = mapParticleClasses.Particles()
    # particles = particlesMCL.particlesMCL()

    
    return particles


def line_segments_intersect(p1, p2, p3, p4):
    """
    Returns True if the line segments defined by the points p1 and p2, and p3 and p4, intersect.
    """
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    x4, y4 = p4

    # Calculate the slopes and y-intercepts of the two lines
    m1 = (y2 - y1) / (x2 - x1) if abs(x2 - x1) > 1e-6 else float('inf')
    b1 = y1 - m1 * x1
    #m2 for wall
    m2 = (y4 - y3) / (x4 - x3) if abs(x4 - x3) > 1e-6 else float('inf')
    b2 = y3 - m2 * x3

    # Check for parallel lines
    if math.isclose(m1, m2):
        return False

    # Calculate the intersection point of the two lines
    if m1 == float('inf'):
        x = x1 # same with x2 as vertical line
        y = m2 * x + b2
    elif m2 == float('inf'):
        x = x3 # same with x4 as vertical line/wall
        y = m1 * x + b1
    else:
        x = (b2 - b1) / (m1 - m2)
        y = m1 * x + b1

    # Check if the intersection point is within the line segments
    x_within_segment_1 = min(x1, x2) - 1e-2 <= x <= max(x1, x2) + 1e-2
    y_within_segment_1 = min(y1, y2) - 1e-2 <= y <= max(y1, y2) + 1e-2
    x_within_segment_2 = min(x3, x4) - 1e-2 <= x <= max(x3, x4) + 1e-2
    y_within_segment_2 = min(y3, y4) - 1e-2 <= y <= max(y3, y4) + 1e-2

    if x_within_segment_1 and y_within_segment_1 and x_within_segment_2 and y_within_segment_2:
        return True
    else:
        return False



def wrap_angle(angle):
    return ((angle + math.pi) % (2*math.pi)) - math.pi

def get_distance_to_wall(map, single_particle):

    '''
    This is a function to get the distance to the wall.
    
    '''
    shortest_distance = 0
    closest_facing_wall = None

    for i in range(len(map.walls)):
        angle_radians = single_particle[2]*math.pi/180
        distance = calculate_distance_to_wall(single_particle[0], single_particle[1], angle_radians, map.walls[i][0], map.walls[i][1], map.walls[i][2], map.walls[i][3])
        wall_1 = (map.walls[i][0], map.walls[i][1])
        wall_2 = (map.walls[i][2], map.walls[i][3])
        particle_1 = (single_particle[0], single_particle[1])
        particle_2 = ( single_particle[0]+(abs(distance)*math.cos(angle_radians)), single_particle[1]+(abs(distance)*math.sin(angle_radians)) )
        # slope = math.tan(single_particle[2]*math.pi/180)

        if distance != float('inf'):
            if line_segments_intersect(particle_1, particle_2, wall_1, wall_2):
                #get shortest distance and identify closest walls. 
                if shortest_distance == 0 and distance >=0:
                    shortest_distance = distance
                    closest_facing_wall = map.walls[i]

                elif shortest_distance > distance and distance >=0:
                    shortest_distance = distance
                    closest_facing_wall = map.walls[i]
            
    # printing the straight path of the particle
    get_cos = shortest_distance*math.cos(single_particle[2]*math.pi/180)
    get_sin = shortest_distance*math.sin(single_particle[2]*math.pi/180)

    particles.drawPath([single_particle[0], single_particle[1], single_particle[0]+get_cos, single_particle[1]+get_sin])   

    print("Particle coordinates: ",single_particle[0]," ", single_particle[1]," ", single_particle[2] )
    print("Shortest distance ", shortest_distance)
    print("Closest Wall: ", closest_facing_wall)

    return shortest_distance, closest_facing_wall

def get_likelihood_function(estimated_distance, measured_distance, standard_deviation):
    ''''
    
    p(z|m) = e^-(z-m)^2/2*(sig(s)^2)
    
    '''
    
    constant = 0.002
    #z is measured distance by sonar and m is estimated distance based on particle set. 

    likelihood = math.exp(-(measured_distance - estimated_distance)**2/ (2*standard_deviation**2)) + constant
    
    return likelihood
    
    
def calculate_likelihood(particle, measured_distance, map):

    #gets distance to closest wall and finds the closest wall
    estimated_distance, closest_wall = get_distance_to_wall(map, particle)
    
    standard_deviation = 2.5 #22.5cm standard deviation
    # Likelihood ready to be multiplied with weight

    likelihood_to_be_multiplied =  get_likelihood_function(estimated_distance, measured_distance, standard_deviation)

    return likelihood_to_be_multiplied
    
    
distance_graphics = 78 # per 10 cm visually
rotation_graphics = -math.pi/2
    
if __name__=="__main__":

    #generates and initializes the map produced. 
    map = initialise_map()
    map.draw()

    particles = initialise_particles()
    particles.update(t=0)
    particles.draw()
    # print(particles)
    # particles.coordinates = np.asarray([[1,10,-90],[10,1,130]])
    # particles.weights = np.asarray([50,50])

    # particles.genNewParticlesRotation(distance_graphics)
    likelihoods = []
    best_particle = {
        'likelihood': 0,
        'particle': None,
        'distance': None
    }
    for i in range(particles.n):
        shortest_distance, closest_wall = get_distance_to_wall(map, particles.data[i])
        # sonar measured distance (real) , example reading 30 cm
        read_val = 30
        likelihood_to_be_multiplied = calculate_likelihood(particles.data[i], read_val, map)
        if best_particle['likelihood'] < likelihood_to_be_multiplied:
            best_particle={
                'likelihood': likelihood_to_be_multiplied,
                'particle': particles.data[i],
                'distance': shortest_distance
            }
        likelihoods.append((likelihood_to_be_multiplied,shortest_distance,particles.data[i]))

    print("Field Order: \n probability, dist, (x,y,theta,weight)")
    print(likelihoods)
    print("---- Sonar expected distance: ", read_val)        
    print("---- Particle with max probability: ", best_particle) 
    print("---- Likelihood is not normalised")       