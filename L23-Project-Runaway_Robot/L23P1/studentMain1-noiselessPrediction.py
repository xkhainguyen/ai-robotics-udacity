# From https://github.com/jeremy-shannon/udacity-AI-for-robotics/blob/master/Project%20-%20Runaway%20Robot/studentMain1-noiselessPrediction.py
from robot import *
from math import *
from matrix import *
import random

def estimate_next_pos(measurement, OTHER = None):
    """Estimate the next (x, y) position of the wandering Traxbot
    based on noisy (x, y) measurements."""
    # assume constant velocity, and a constant change in angle

    if not OTHER:
        OTHER = []
    avgdist = 0
    avgdtheta = 0
    OTHER.append(measurement)
    dist = []
    theta = []
    if len(OTHER) > 1:
        for i in range(len(OTHER)-1):
            dist.append(distance_between(OTHER[i+1],OTHER[i]))
            theta.append(atan2(OTHER[i+1][1]-OTHER[i][1], OTHER[i+1][0]-OTHER[i][0]))
    else:
        dist = [0]
        theta = [0]
    avgdist = sum(dist) / float(len(dist))
    dtheta = []
    if len(theta) > 1:
        for i in range(len(theta)-1):
            dtheta.append((theta[i+1]-theta[i])%(2*pi))
        avgdtheta = sum(dtheta) / float(len(dtheta))

    xy_estimate = (measurement[0]+avgdist*cos(theta[-1]+avgdtheta), measurement[1]+avgdist*sin(theta[-1]+avgdtheta))

    # You must return xy_estimate (x, y), and OTHER (even if it is None) 
    # in this order for grading purposes.
    return xy_estimate, OTHER 

# A helper function you may find useful.
def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def demo_grading(estimate_next_pos_fcn, target_bot, OTHER = None):
    localized = False
    distance_tolerance = 0.01 * target_bot.distance
    ctr = 0
    # if you haven't localized the target bot, make a guess about the next
    # position, then we move the bot and compare your guess to the true
    # next position. When you are close enough, we stop checking.
    #For Visualization
    import turtle    #You need to run this locally to use the turtle module
    window = turtle.Screen()
    window.bgcolor('white')
    size_multiplier= 25.0  #change Size of animation
    broken_robot = turtle.Turtle()
    broken_robot.shape('turtle')
    broken_robot.color('green')
    broken_robot.resizemode('user')
    broken_robot.shapesize(0.35, 0.35, 0.35)
    measured_broken_robot = turtle.Turtle()
    measured_broken_robot.shape('circle')
    measured_broken_robot.color('red')
    measured_broken_robot.resizemode('user')
    measured_broken_robot.shapesize(0.2, 0.2, 0.2)
    prediction = turtle.Turtle()
    prediction.shape('arrow')
    prediction.color('blue')
    prediction.resizemode('user')
    prediction.shapesize(0.4, 0.4, 0.4)
    prediction.penup()
    broken_robot.penup()
    measured_broken_robot.penup()
    #End of Visualization
    while not localized and ctr <= MAX_CTR:
        ctr += 1
        print("Result #", ctr)
        measurement = target_bot.sense()
        position_guess, OTHER = estimate_next_pos_fcn(measurement, OTHER)
        target_bot.move_in_circle()
        true_position = (target_bot.x, target_bot.y)
        error = distance_between(position_guess, true_position)
        print("=> Compare: ", position_guess, true_position, error)
        if error <= distance_tolerance:
            print ("You got it right! It took you ", ctr, " steps to localize.")
            # localized = True
        if ctr == MAX_CTR:
            print ("Sorry, it took you too many steps to localize the target.")
        #More Visualization
        measured_broken_robot.setheading(target_bot.heading*180/pi)
        measured_broken_robot.goto(measurement[0]*size_multiplier, measurement[1]*size_multiplier-200)
        measured_broken_robot.stamp()
        broken_robot.setheading(target_bot.heading*180/pi)
        broken_robot.goto(target_bot.x*size_multiplier, target_bot.y*size_multiplier-200)
        broken_robot.stamp()
        prediction.setheading(target_bot.heading*180/pi)
        prediction.goto(position_guess[0]*size_multiplier, position_guess[1]*size_multiplier-200)
        prediction.stamp()
        #End of Visualization
    return localized



# This is how we create a target bot. Check the robot.py file to understand
# How the robot class behaves.
test_target = robot(2.1, 4.3, 0.5, 2*pi / 34.0, 1.5)
test_target.set_noise(0.0, 0.0, 0.0)
W_SIZE = 5
MAX_CTR = 1000
demo_grading(estimate_next_pos, test_target)