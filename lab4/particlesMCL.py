import math
import random
import numpy as np
import time

NUMBER_OF_PARTICLES = 100


class particlesMCL():
    """

    Class for particles. Practical 2.1 - Representing and Displaying Uncertain Motion with a Particle Set
    
    """

    def __init__(self):
        # Coordinates (x, y, theta)
        self.coordinates = np.zeros((NUMBER_OF_PARTICLES, 3))
        self.weights = np.full((NUMBER_OF_PARTICLES, 1), 1/NUMBER_OF_PARTICLES)

        #sets the coordinates to have value 0 and also initialize weights to have little biases. 
        for i in range(NUMBER_OF_PARTICLES):
            self.coordinates[i][0] = 0
            self.coordinates[i][1] = 0
            self.coordinates[i][2] = 0
            self.weights[i] = 1/NUMBER_OF_PARTICLES
        
        map_size = 210
        self.map_size    = map_size;    # in cm;
        self.canvas_size = 768;         # in pixels;
        self.margin      = 0.05*map_size;
        self.scale       = self.canvas_size/(map_size+2*self.margin);

    def initialise_at(self, x, y):
        # initialise the particles at the starting coordinates
        for i in range(NUMBER_OF_PARTICLES):
            self.coordinates[i][0] = x
            self.coordinates[i][1] = y
            self.coordinates[i][2] = 0
            self.weights[i] = 1/NUMBER_OF_PARTICLES

    def genNewParticlesStraight(self, D):
        """

        Generate new particles from a straight line motion model.
        params: D

        """

        mu = 0
        sigma_e = D*0.025 #initially 0.05    
        sigma_f = 0.025 #initially 0.05
        
        particles = []

        for i in range(NUMBER_OF_PARTICLES):
            #gets e and f parameters from a normal distribution. 
            e = random.gauss(mu, sigma_e)
            f = random.gauss(mu, sigma_f)

            x_new = self.coordinates[i][0] + (D+e) * math.cos(float(self.coordinates[i][2])*math.pi/180)
            y_new = self.coordinates[i][1] + (D+e) * math.sin(float(self.coordinates[i][2])*math.pi/180)
            theta_new = self.coordinates[i][2] + f
            particles.append((x_new, y_new, theta_new))

            # self.printLine((self.coordinates[i][0], self.coordinates[i][1], x_new, y_new)) # to be expected path
            self.coordinates[i][0] = x_new
            self.coordinates[i][1] = y_new
            self.coordinates[i][2] = theta_new
        
        self.printParticles(particles)


    def genNewParticlesRotation(self, alpha):
        """Generate new particles from a rotation motion model.
            params: x, y, theta, alpha
            return: None
        """
        mu = 0
        sigma_g = 0.00125*alpha #inititally 0.0025
        g = random.gauss(mu, sigma_g)

        particles = []
        for i in range(NUMBER_OF_PARTICLES):    # _ is cooler than i when i is not used
            x_new = self.coordinates[i][0]
            y_new = self.coordinates[i][1]

            theta_new = self.coordinates[i][2] + alpha + g
            theta_new = self.wrapAngleTo180(theta_new)
            particles.append((x_new, y_new, theta_new))

            self.coordinates[i][0] = x_new
            self.coordinates[i][1] = y_new
            self.coordinates[i][2] = theta_new
            
        self.printParticles(particles)

    def wrapAngleTo180(self, theta_new):
        if theta_new>180:
            theta_new -=360

        elif theta_new<-180:
            theta_new+=360

        return theta_new



    def printLine(self, line):
        """Print a line to the screen."""
        # line is a tuple (x0, y0, x1, y1)
        shifted_line = list(line)
        shifted_line[0] += 150
        shifted_line[1] += 700
        shifted_line[2] += 150
        shifted_line[3] += 700
        line = tuple(shifted_line)
        print("drawLine:"+ str(line))

    def convertNPtoTuples(self, particlesNP):
        particles = []
        for i in range(len(particlesNP.coordinates)):
            particles.append((particlesNP.coordinates[i][0], particlesNP.coordinates[i][1], float(particlesNP.coordinates[i][2])))
        return particles

    def printParticles(self, particles): # not used, trying to shift and scale by canvas size
        """Print particles to the screen."""

        # list of 3-tuples (x,y, theta)
        for i in range(len(particles)):
            new_tuple = list(particles[i])
            new_tuple[0] = self.__screenX(new_tuple[0])
            new_tuple[1] = self.__screenY(new_tuple[1])
            # new_tuple[2] += new_tuple[2] # not sure if we add the rotation too
            #  new_tuple[2] = new_tuple[2][0]
            particles[i] = (new_tuple[0], new_tuple[1], float(new_tuple[2]))
            #print(particles[i])
        # print("test: "+str(particles))
        print("drawParticles:"+ str(particles))
        #x, w = initialise_particles()


    # def update_particles(self, coordinates, weights, motion):
    def update_particles(self, motion, angle):
        """ update particles by ..."""
        # 4 per every side and 1 per rotation => 20
        for _ in range(4):
            for _ in range(4):
                self.genNewParticlesStraight(motion)
                time.sleep(1)
            #print(self.coordinates.size)
            self.genNewParticlesRotation(angle) # turn 90 degrees left ?? degrees or rad??
            time.sleep(1)

    def drawPath(self,line):
        canvas.drawLine(line);
    def __screenX(self,x):
        return (x + self.margin)*self.scale
    def __screenY(self,y):
        return (self.map_size + self.margin - y)*self.scale

# A Canvas class for drawing a map and particles:
#     - it takes care of a proper scaling and coordinate transformation between
#      the map frame of reference (in cm) and the display (in pixels)
class Canvas:
    def __init__(self,map_size=210):
        self.map_size    = map_size;    # in cm;
        self.canvas_size = 768;         # in pixels;
        self.margin      = 0.05*map_size;
        self.scale       = self.canvas_size/(map_size+2*self.margin);

    def drawLine(self,line):
        x1 = self.__screenX(line[0]);
        y1 = self.__screenY(line[1]);
        x2 = self.__screenX(line[2]);
        y2 = self.__screenY(line[3]);
        print ("drawLine:" + str((x1,y1,x2,y2)))

    def drawParticles(self,data):
        display = [(self.__screenX(d[0]),self.__screenY(d[1])) + d[2:] for d in data];
        print ("drawParticles:" + str(display))

    def __screenX(self,x):
        return (x + self.margin)*self.scale

    def __screenY(self,y):
        return (self.map_size + self.margin - y)*self.scale

# A Map class containing walls
class Map:
    def __init__(self):
        self.walls = [];

    def add_wall(self,wall):
        self.walls.append(wall);

    def clear(self):
        self.walls = [];

    def draw(self):
        for wall in self.walls:
            canvas.drawLine(wall);

canvas = Canvas() # setting up the canvas to draw the map

# particles = particles()
# particles.update_particles(155, -math.pi/2)