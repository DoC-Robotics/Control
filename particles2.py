import math
import random
import numpy as np
import time

NUMBER_OF_PARTICLES = 100


class particles():
    """Class for particles. Practical 2.1 - Representing and Displaying Uncertain Motion with a Particle Set"""
    def __init__(self):
        # Coordinates (x, y, theta)
        self.coordinates = np.zeros((NUMBER_OF_PARTICLES, 3))
        self.weights = np.full((NUMBER_OF_PARTICLES, 1), 1/NUMBER_OF_PARTICLES)
        # for i in range(NUMBER_OF_PARTICLES):
        #     self.coordinates[i][0] = 0
        #     self.coordinates[i][1] = 0
        #     self.coordinates[i][2] = 0
            #self.weights[i] = 1/NUMBER_OF_PARTICLES


    def genNewParticlesStraight(self, D):
        """Generate new particles from a straight line motion model.
            params: D
        """
        mu = 0
        sigma_e = D*0.01   #CHANGE - 1 HOUR LEFT TO DO IT ELSE I WILL REPORT YOU TO TICKETING TEAM
        sigma_f = 0.02   #CHANGE - 1 HOUR LEFT TO DO IT ELSE I WILL REPORT YOU TO TICKETING TEAM
        
        particles = []
        for i in range(NUMBER_OF_PARTICLES):
            
            e = random.gauss(mu, sigma_e)
            f = random.gauss(mu, sigma_f)

            x_new = self.coordinates[i][0] + (D+e) * math.cos(self.coordinates[i][2])
            y_new = self.coordinates[i][1] + (D+e) * math.sin(self.coordinates[i][2])
            theta_new = self.coordinates[i][2] + f
            particles.append((x_new, y_new, theta_new))
            #self.printLine((self.coordinates[i][0], self.coordinates[i][1], x_new, y_new)) # line from initial mean coords to where??
            self.coordinates[i][0] = x_new
            self.coordinates[i][1] = y_new
            self.coordinates[i][2] = theta_new
        
        self.printParticles(particles)


    def genNewParticlesRotation(self, x, y, theta, alpha):
        """Generate new particles from a rotation motion model.
            params: x, y, theta, alpha
            return: None
        """
        mu = 0
        sigma_g = 1
        g = random.gauss(mu, sigma_g)

        particles = []
        for _ in range(NUMBER_OF_PARTICLES):    # _ is cooler than i when i is not used
            x_new = x
            y_new = y
            theta_new = theta + alpha + g
            particles.append((x_new, y_new, theta_new))

        self.printParticles(particles)


    def printLine(self, line):
        """Print a line to the screen."""
        # line is a tuple (x0, y0, x1, y1)
        print("drawLine:"+ str(line))


    def printParticles(self, particles):
        """Print particles to the screen."""
        # list of 3-tuples (x,y, theta)
        for i in range(len(particles)):
            new_tuple = list(particles[i])
            new_tuple[0] += 100
            new_tuple[1] += 300
            particles[i] = tuple(new_tuple)
            print(particles[i])
        print("drawParticles:"+ str(particles))
        #x, w = initialise_particles()


    # def update_particles(self, coordinates, weights, motion):
    def update_particles(self, motion):
        """ update particles by ..."""
        # 4 per every side and 1 per rotation => 20
        for _ in range(1):
            for _ in range(4):
                self.genNewParticlesStraight(motion)
                time.sleep(1)
            print(self.coordinates.size)
            #self.genNewParticlesRotation(   self.coordinates[self.coordinates.size-1][0], 
            #                                self.coordinates[self.coordinates.size-1][1], 
            #                                self.coordinates[self.coordinates.size-1][2], 
            #                                90) # turn 90 degrees left ?? degrees or rad??


particles = particles()
particles.update_particles(100)