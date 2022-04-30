from robot import *
from math import *
from copy import deepcopy
import random

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
                pos = [x / self.N, y / self.N, heading / self.N]
                vel = [turning/self.N, distance/self.N]
            return [pos, vel]

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
        # measurement probability
        # 
        def measurement_prob(self, robot, measurement):
            m_noise = 0.5

            # compute errors
            error_x = measurement[0] - robot.x
            # print(error_x)
            error_y = measurement[1] - robot.y

            # calculate Gaussian
            error = exp(- (error_x ** 2) / (m_noise ** 2) / 2.0) \
                / sqrt(2.0 * pi * (m_noise ** 2))
            error *= exp(- (error_y ** 2) / (m_noise ** 2) / 2.0) \
                / sqrt(2.0 * pi * (m_noise ** 2))
            # print(error)

            return error


        # --------
        #
        # sensing and resampling
        # 

        def sense(self, Z):
            w = []
            for i in range(self.N):
                # print(self.data[i])
                w.append(1*self.measurement_prob(self.data[i], Z))
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

