# LECTURE 23: PROJECT RUNAWAY ROBOT, PART 1: NOISELESS PREDICTION
# By Xuan Khai Nguyen
#####################
# This program uses Particle Filter to estimate the distance and turning angle of the robot,
# hence, predict the next position. The filter creates N_PARTICLE (a very great number) clones 
# of the robot, each is initialized with: 
# 1) Initial position is taken from the first measuarement
# 2) Heading angle is randomly created in the range (see in pFilter.py)
# 3) Distance and turning angle are randomly created in the range (see in pFilter.py)
# The filter goes through rolling out, resampling and averaging to return the estimated variables.
# Using the last measurement and estimated variables, one-step position prediction is made.

from robot import *  # Check the robot.py tab to see how this works.
from math import *
from matrix import * # Check the matrix.py tab to see how this works.
from pFilter import *

def estimate_next_pos(measurement, OTHER = None):
    if not OTHER:   
        pf = particles(measurement[0], measurement[1], 0., 0., 0., N_PARTICLE)
    else:
        pf = OTHER
        pf.move()
        pf.sense(measurement)
    # estimate
    pos, vel = pf.get_data()
    print(f"Estimated heading = {pos[2]}, vel = {(vel[0],vel[1])}")
    # prediction
    predictor = robot(measurement[0], measurement[1], pos[2], vel[0], vel[1])
    predictor.set_noise(0.0, 0.0, 0.0) 
    predictor.move(vel[0], vel[1])
    # output
    xy_estimate = (predictor.x, predictor.y)
    OTHER = pf 
    return xy_estimate, OTHER 

# A helper function you may find useful.
def distance_between(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# This is here to give you a sense for how we will be running and grading
# your code. Note that the OTHER variable allows you to store any 
# information that you want. 
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
    while not localized and ctr <= 30:
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
            localized = True
        if ctr == 1000:
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

# MAIN 
test_target = robot(2.1, 4.3, 0.5, 2*pi / 34.0, 1.5)
test_target.set_noise(0.0, 0.0, 0.0)
N_PARTICLE = 100000
demo_grading(estimate_next_pos, test_target)




