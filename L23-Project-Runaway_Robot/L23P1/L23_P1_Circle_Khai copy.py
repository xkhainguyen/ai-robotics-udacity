# LECTURE 23: PROJECT RUNAWAY ROBOT, PART 1: NOISELESS PREDICTION
# By Xuan Khai Nguyen
#####################
# This program uses normal geometry to estimate the distance and turning angle of the robot,
# hence, predict the next position. The approach calculates the distance of two points and 
# the turning angle by the difference between two distance vector. Therefore it requires at 
# least three points noiseless case and average running windows in noise case.
#####################
# Solution from https://discussions.udacity.com/t/heres-my-solution-for-project1-share-yours-too/119881
# Problem: Quadrant

from robot import *  # Check the robot.py tab to see how this works.
from math import *
from matrix import * # Check the matrix.py tab to see how this works.

def estimate_next_pos(measurement, OTHER = None):
    """Estimate the next (x, y) position of the wandering Traxbot
    based on noisy (x, y) measurements."""
    if OTHER == None:
        OTHER = {'xy':measurement} #for tracking the previous (x,y)
        xy_estimate = (0.0,0.0)
    else:
        distance = distance_between(OTHER['xy'],measurement) #distance between the 2 points
        heading = acos((measurement[0]-OTHER['xy'][0])/distance) #finding the heading direction with some basic trigonometry
        # Problem: Quadrant
        if 'heading' not in OTHER.keys():            
            turning = None
            xy_estimate=(0.,0.)
        else:
            turning = -1 * (OTHER['heading'] - heading) # Finding the turning direction, the -1 is just for bug fixing
            # Now that we've figured out the heading, turning and the distance
            # Lets find the next point by creating a dummy robot
            dummy = robot(measurement[0], measurement[1], heading, turning, distance)
            dummy.set_noise(0., 0., 0.)
            dummy.move_in_circle()
            xy_estimate = (dummy.x, dummy.y)
        OTHER['heading'] = heading
        OTHER['xy'] = measurement
    # You must return xy_estimate (x, y), and OTHER (even if it is None) 
    # in this order for grading purposes.
    return xy_estimate, OTHER

# A helper function you may find useful.
def distance_between(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def demo_grading(estimate_next_pos_fcn, target_bot, OTHER = None):
    localized = False
    distance_tolerance = 0.01 * target_bot.distance
    ctr = 0

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

# MAIN 
test_target = robot(2.1, 4.3, 0.5, 2*pi / 34.0, 1.5)
test_target.set_noise(0.0, 0.0, 0.0)
W_SIZE = 5
MAX_CTR = 1000
demo_grading(estimate_next_pos, test_target)