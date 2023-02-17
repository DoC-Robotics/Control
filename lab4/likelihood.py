import math
import mapParticleClasses



def calculate_distance_to_wall(x_coor, y_coor, theta, wall_a_x_coor,wall_a_y_coor, wall_b_x_coor, wall_b_y_coor):
    '''
    m =(By - Ay )(Ax - x) - (Bx - Ax )(Ay - y)/ (By - Ay ) cos θ - (Bx - Ax ) sin θ
    Gets distance to wall based on x,y, theta parameters.  
    '''
    distance = ((wall_b_y_coor-wall_a_y_coor) * (wall_a_x_coor - x_coor) - (wall_b_x_coor - wall_a_x_coor) * (wall_a_y_coor - y_coor))/((wall_b_y_coor - wall_a_y_coor)*math.cos(theta) - (wall_b_x_coor - wall_a_x_coor) * math.sin(theta))
    return distance

def initialise_map():
    mymap = mapParticleClasses.Map();
    # Definitions of walls
    # a: O to A
    # b: A to B
    # c: C to D
    # d: D to E
    # e: E to F
    # f: F to G
    # g: G to H
    # h: H to O
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
    particles = mapParticleClasses.Particles()
    return particles

def get_distance_to_closest_wall(map, single_particle):
    shortest_distance = 0
    closest_wall = None
    for i in range(len(map.walls)):
        distance = calculate_distance_to_wall(single_particle[0], single_particle[1], single_particle[2], map.walls[i][0], map.walls[i][1], map.walls[i][2], map.walls[i][3])
        if shortest_distance == 0 and distance >=0:
            shortest_distance = distance
            closest_wall = map.walls[i]
        elif shortest_distance > distance and distance >=0:
            shortest_distance = distance
            closest_wall = map.walls[i]
    print(shortest_distance)
    print(closest_wall)
    return shortest_distance, closest_wall

def get_likelihood_function(estimated_distance, measured_distance, standard_deviation):
    constant = 0.2
    likelihood = math.exp(-(measured_distance - estimated_distance)**2/ (2*standard_deviation**2)) + constant
    return likelihood
    
    
def calculate_likelihood(particle, measured_distance, map):
    estimated_distance, closest_wall = get_distance_to_closest_wall(map, particle)
    
    standard_deviation = 2.5 #2.5cm standard deviation
    # Likelihood ready to be multiplied with weight
    likelihood_to_be_multiplied =  get_likelihood_function(estimated_distance, measured_distance, standard_deviation)
    return likelihood_to_be_multiplied
    

# def test_distance_to_closest_wall(map, particles):
#     '''
#     function to test printing distance to wall.
#     '''
#     for i in range(particles.n):
#         shortest_distance = 0
#         for j in range(len(map.walls)):
#             distance = calculate_distance_to_wall(particles.data[i][0], particles.data[i][1], particles.data[i][2], map.walls[j][0], map.walls[j][1], map.walls[j][2], map.walls[j][3])
#             if shortest_distance == 0 and distance >=0:
#                 shortest_distance = distance
#             elif shortest_distance > distance and distance >=0:
#                 shortest_distance = distance
#             print(distance)
#             print("Wall coordinates: ", map.walls[j][0]," ", map.walls[j][1]," ", map.walls[j][2]," ", map.walls[j][3])
#         print("Particle coordinates: ",particles.data[i][0]," ", particles.data[i][1]," ", particles.data[i][2] )
#         print("shortest distance: ", shortest_distance)
    
    
if __name__=="__main__":
    map = initialise_map()
    particles = initialise_particles()
    particles.update(t=0)
    for i in range(particles.n):
        shortest_distance, closest_wall = get_distance_to_closest_wall(map, particles.data[i])
        
    