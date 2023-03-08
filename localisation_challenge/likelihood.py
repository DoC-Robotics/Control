import math
import particlesMCL

import numpy as np

# Likelihood for lab3.2 and 3.3


def calculate_distance_to_wall(
    x_coor, y_coor, theta, wall_a_x_coor, wall_a_y_coor, wall_b_x_coor, wall_b_y_coor
):
    """

    m ={(By - Ay )(Ax - x) - (Bx - Ax )(Ay - y)}/{(By - Ay) cos θ - (Bx - Ax) sin θ}
    Gets distance to wall based on x,y, theta parameters.

    """
    # if ((theta == 90 or theta == 270) and wall_b_x_coor-wall_a_x_coor == 0):
    #     return float('inf')
    # elif ((theta == 0 or theta == 180 or theta == 360) and wall_b_y_coor-wall_a_y_coor == 0):
    #     return float('inf')
    # division by 0 if theta is 90 and wall y is on x axis (0 coords), same problem if 180 theta with wall on y axis (0 coords)
    # parallel case when walls starting from origin
    # (0 - 0)*1 - (168-0)*0 = 0 -> particle with orientation +-90degreees, and wall OA
    # (210-0)*0 - (0 - 0)*1 -> particle with orientation +-180degrees, and wall OH

    # gives the numerator and denominator for the distance to the wall.
    numerator = (wall_b_y_coor - wall_a_y_coor) * (wall_a_x_coor - x_coor) - (
        wall_b_x_coor - wall_a_x_coor
    ) * (wall_a_y_coor - y_coor)
    denominator = (wall_b_y_coor - wall_a_y_coor) * math.cos(theta) - (
        wall_b_x_coor - wall_a_x_coor
    ) * math.sin(theta)

    if denominator == 0:
        return float("inf")
    distance = (numerator) / (denominator)

    return distance


def initialise_map():
    mymap = particlesMCL.Map()
    # Definitions of walls
    """
    
    a: O to A, b: A to B, c: C to D, d: D to E
    e: E to F, f: F to G, g: G to H ,h: H to O
    
    """
    mymap.add_wall((0, 0, 0, 168))
    # a (wall OA)
    mymap.add_wall((0, 168, 84, 168))
    # b (wall OH)
    mymap.add_wall((84, 126, 84, 210))
    # c  (CD)
    mymap.add_wall((84, 210, 168, 210))
    # d (BD)
    mymap.add_wall((168, 210, 168, 84))
    # e -> we use a particle and will calculate the lowest distance.
    mymap.add_wall((168, 84, 210, 84))
    # f
    mymap.add_wall((210, 84, 210, 0))
    # g
    mymap.add_wall((210, 0, 0, 0))
    # h
    return mymap


def initialise_particles():
    # initializes particles and returns these.
    # particles = mapParticleClasses.Particles()
    particles = particlesMCL.particlesMCL()

    return particles


def is_target_on_wall(particle, theta, wall_a, wall_b):
    x, y = particle
    theta = theta
    wall_a_x, wall_a_y = wall_a
    wall_b_x, wall_b_y = wall_b

    t_numerator = math.sin(theta) * (wall_a_x - x) - math.cos(theta) * (wall_a_y - y)
    t_denominator = math.cos(theta) * (wall_b_y - wall_a_y) - math.sin(theta) * (
        wall_b_x - wall_a_x
    )
    if t_denominator != 0:
        t = t_numerator / t_denominator
    else:
        return False

    return 0 <= t <= 1


def wrap_angle(angle):
    return ((angle + math.pi) % (2 * math.pi)) - math.pi


def get_distance_to_wall(map, particles, particle_index):
    """
    This is a function to get the distance to the wall.

    """
    shortest_distance = 0
    closest_facing_wall = None

    for i in range(len(map.walls)):
        angle_radians = particles.coordinates[particle_index][2] * math.pi / 180
        distance = calculate_distance_to_wall(
            particles.coordinates[particle_index][0],
            particles.coordinates[particle_index][1],
            angle_radians,
            map.walls[i][0],
            map.walls[i][1],
            map.walls[i][2],
            map.walls[i][3],
        )
        wall_1 = (map.walls[i][0], map.walls[i][1])
        wall_2 = (map.walls[i][2], map.walls[i][3])
        particle_1 = (
            particles.coordinates[particle_index][0],
            particles.coordinates[particle_index][1],
        )

        if distance != float("inf"):
            if is_target_on_wall(particle_1, angle_radians, wall_1, wall_2):
                # get shortest distance and identify closest walls.
                if shortest_distance == 0 and distance >= 0:
                    shortest_distance = distance
                    closest_facing_wall = map.walls[i]

                elif shortest_distance > distance and distance >= 0:
                    shortest_distance = distance
                    closest_facing_wall = map.walls[i]

    # # printing the straight path of the particle
    # get_cos = shortest_distance*math.cos(particles.coordinates[particle_index][2] * math.pi/180)
    # get_sin = shortest_distance*math.sin(particles.coordinates[particle_index][2] * math.pi/180)

    # particles.drawPath((particles.coordinates[particle_index][0], particles.coordinates[particle_index][1], particles.coordinates[particle_index][0]+get_cos, particles.coordinates[particle_index][1]+get_sin))

    # print("Particle coordinates: ",particles.coordinates[particle_index][0]," ", particles.coordinates[particle_index][1]," ", particles.coordinates[particle_index][2] )
    # print("Shortest distance ", shortest_distance)
    # print("Closest Wall: ", closest_facing_wall)

    return shortest_distance


def get_likelihood_function(estimated_distance, measured_distance, standard_deviation):
    """'

    p(z|m) = e^-(z-m)^2/2*(sig(s)^2)

    """

    constant = 0.05
    # z is measured distance by sonar and m is estimated distance based on particle set.

    likelihood = (
        math.exp(
            -((measured_distance - estimated_distance) ** 2)
            / (2 * standard_deviation**2)
        )
        + constant
    )

    return likelihood


def calculate_likelihood(particles, particle_index, measured_distance, map):
    # gets distance to closest wall and finds the closest wall
    estimated_distance = get_distance_to_wall(map, particles, particle_index)

    standard_deviation = 2.5  # 22.5cm standard deviation
    # Likelihood ready to be multiplied with weight

    likelihood_to_be_multiplied = get_likelihood_function(
        estimated_distance, measured_distance, standard_deviation
    )

    return likelihood_to_be_multiplied


distance_graphics = 78  # per 10 cm visually
rotation_graphics = -math.pi / 2


def update_particles_weights(particles, distance_measurement, map):
    for i in range(particlesMCL.NUMBER_OF_PARTICLES):
        particles.weights[i] = particles.weights[i] * calculate_likelihood(
            particles, i, distance_measurement, map
        )
    return particles


if __name__ == "__main__":
    # generates and initializes the map produced.
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
    best_particle = {"likelihood": 0, "particle": None, "distance": None}
    for i in range(particles.n):
        shortest_distance = get_distance_to_wall(map, particles.data[i])
        # sonar measured distance (real) , example reading 30 cm
        read_val = 30
        likelihood_to_be_multiplied = calculate_likelihood(
            particles.data[i], read_val, map
        )
        if best_particle["likelihood"] < likelihood_to_be_multiplied:
            best_particle = {
                "likelihood": likelihood_to_be_multiplied,
                "particle": particles.data[i],
                "distance": shortest_distance,
            }
        likelihoods.append(
            (likelihood_to_be_multiplied, shortest_distance, particles.data[i])
        )

    print("Field Order: \n probability, dist, (x,y,theta,weight)")
    print(likelihoods)
    print("---- Sonar expected distance: ", read_val)
    print("---- Particle with max probability: ", best_particle)
    print("---- Likelihood is not normalised")
