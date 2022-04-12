from math import *
from copy import deepcopy
import random

# helper function to map all angles onto [-pi, pi]
def angle_trunc(a):
    while a < 0.0:
        a += pi * 2
    return ((a + pi) % (pi * 2)) - pi

class robot:

    def __init__(self, x = 0.0, y = 0.0, heading = 0.0, turning = 2*pi/10, distance = 1.0):
        """This function is called when you create a new robot. It sets some of 
        the attributes of the robot, either to their default values or to the values
        specified when it is created."""
        self.x = x
        self.y = y
        self.heading = heading
        self.turning = turning # only applies to target robots who constantly move in a circle
        self.distance = distance # only applies to target bot, who always moves at same speed.
        self.turning_noise    = 0.0
        self.distance_noise    = 0.0
        self.measurement_noise = 0.0
    def set_init(self, x, y):
        self.x = x
        self.y = y
        self.heading = 0.5
    def set_noise(self, new_t_noise, new_d_noise, new_m_noise):
        """This lets us change the noise parameters, which can be very
        helpful when using particle filters."""
        self.turning_noise     = float(new_t_noise)
        self.distance_noise    = float(new_d_noise)
        self.measurement_noise = float(new_m_noise)


    def move(self, turning, distance, tolerance = 0.001, max_turning_angle = pi):
        """This function turns the robot and then moves it forward."""

        # truncate to fit physical limitations
        turning = max(-max_turning_angle, turning)
        turning = min( max_turning_angle, turning)
        distance = max(0.0, distance)

        # Execute motion
        self.heading += turning
        self.heading = angle_trunc(self.heading)
        self.x += distance * cos(self.heading)
        self.y += distance * sin(self.heading)

    def move_in_circle(self):
        """This function is used to advance the runaway target bot."""
        self.move(self.turning, self.distance)

    def sense(self):
        """This function represents the robot sensing its location. When
        measurements are noisy, this will return a value that is close to, 
        but not necessarily equal to, the robot's (x, y) position."""
        return (self.x, self.y)

    def __repr__(self):
        """This allows us to print a robot's position"""
        return '[%.5f, %.5f]'  % (self.x, self.y)
        # return "[{,.5f}, {,.5f}]".format(self.x, self.y)

    def measurement_prob(self, measurement):
        m_noise = 0.5

        # compute errors
        error_x = measurement[0] - self.x
        # print(error_x)
        error_y = measurement[1] - self.y

        # calculate Gaussian
        error = exp(- (error_x ** 2) / (m_noise ** 2) / 2.0) \
            / sqrt(2.0 * pi * (m_noise ** 2))
        error *= exp(- (error_y ** 2) / (m_noise ** 2) / 2.0) \
            / sqrt(2.0 * pi * (m_noise ** 2))
        # print(error)

        return error


class particles:

        # --------
        # init: 
        #   creates particle set with given initial position
        #

        def __init__(self, x, y, 
                     turning_noise, distance_noise, measurement_noise, N = 1000):
            self.N = N
            self.turning_noise    = turning_noise
            self.distance_noise    = distance_noise
            self.measurement_noise = measurement_noise
            
            self.data = []
            for i in range(self.N):
                r = robot(x, y, heading = random.random()*2*pi - pi, 
                        turning = random.random()*2*pi/4 - pi/4, distance = random.random()*5)
                # r = robot(x, y, heading = random.random()*2*pi - pi, 
                #         turning = random.random()*2*pi - pi, distance = random.random()*5)
                # r.set_noise(turning_noise, distance_noise, measurement_noise)
                self.data.append(r)


        # --------
        #
        # extract data from a particle set
        # 
        
        def get_data(self):
            x = 0.0
            y = 0.0
            heading = 0.0
            turning = 0.
            distance = 0.
            for i in range(self.N):
                x += self.data[i].x
                y += self.data[i].y
                # orientation is tricky because it is cyclic. By normalizing
                # around the first particle we are somewhat more robust to
                # the 0=2pi problem
                heading  += self.data[i].heading
                turning  += self.data[i].turning
                distance += self.data[i].distance
                # pos = [x / self.N, y / self.N, heading / self.N]
                vel = [heading/self.N, turning/self.N, distance/self.N]
            return vel

        # --------
        #
        # motion of the particles
        # 

        def move(self):
            # newdata = []

            for i in range(self.N):
                self.data[i].move(self.data[i].turning, self.data[i].distance)

        # --------
        #
        # sensing and resampling
        # 

        def sense(self, Z):
            w = []
            for i in range(self.N):
                # print(self.data[i])
                w.append(1*self.data[i].measurement_prob(Z))
                # print(f"estimated distance {self.data[i].distance}, power is {w[i]}, real one is 1.5")
            # resampling (careful, this is using shallow copy)
            p3 = [None]*self.N
            index = int(random.random() * self.N)
            beta = 0.0
            mw = max(w)
            # print(f"max w = {mw}")

            for i in range(self.N):
                beta += random.random() * 2 * mw
                while beta > w[index]:
                    beta -= w[index]
                    index = (index + 1) % self.N
                p3[i] = deepcopy(self.data[index])
                # must be deepcopy due to object
                # print(f"estimated distance {self.data[index].distance}, power is {w[index]}, real one is 1.5")
            self.data = deepcopy(p3)

