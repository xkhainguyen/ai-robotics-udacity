# LECTURE 23: PROJECT RUNAWAY ROBOT, PART 1: NOISELESS PREDICTION
# By Xuan Khai Nguyen
#####################
# This program uses normal geometry to estimate the distance and turning angle of the robot,
# hence, predict the next position. The approach calculates the distance of two points and 
# the turning angle by the difference between two distance vector. Therefore it requires at 
# least three points noiseless case and average running windows in noise case.
#####################

from robot import *  # Check the robot.py tab to see how this works.
from math import *
from matrix import * # Check the matrix.py tab to see how this works.

def estimate_next_pos(measurement, OTHER = None):
    if not OTHER:
        t = 0
        xy_estimate = measurement
        r_ls = []
        dtheta_ls = []
        theta0_ls = []
        vector_pre = []
    else:
        t, pre, vector_pre, r_ls, dtheta_ls, theta0_ls = OTHER
        t += 1
        print("t = ",t)

        # x_ls.append(measurement[0])
        # y_ls.append(measurement[1])
        # xc = sum(x_ls)/len(x_ls)
        # yc = sum(y_ls)/len(y_ls)
        # print("Center = ", xc, yc)
        # xc = -2.43
        # yc = 11.043
        
        # r = distance_between(measurement, [xc, yc])
        # r_ls.append(r)
        # r = sum(r_ls)/len(r_ls)
        # print("R = ",r)
        # r = 8.12

        vector = [measurement[0] - pre[0], measurement[1] - pre[1]]

        if t >= 2:
            dtheta = vector[0]*vector_pre[0] + vector[1]*vector_pre[1]
            dtheta = dtheta/sqrt(vector[0]**2+vector[1]**2) 
            dtheta = dtheta/sqrt(vector_pre[0]**2+vector_pre[1]**2)   
            # if dtheta > 1:
            #     dtheta = 1
            # if dtheta < -1:
            #     dtheta = -1 
            dtheta = acos(dtheta)%(2*pi)
            dtheta_ls.append(dtheta)
            dtheta = sum(dtheta_ls)/len(dtheta_ls)
            print("dtheta = ", dtheta)
            # dtheta = 0.1848
        
            r = 0.5*distance_between(measurement, pre)/sin(dtheta/2)
            print("R = ",r)

            # theta0 = atan2(vector[1], vector[0])
            # theta0 = (theta0 - t*dtheta)
            # theta0_ls.append(theta0)
            # theta0 = sum(theta0_ls)/len(theta0_ls)

            theta0 = measurement[0] - pre[0]
            theta0 = (theta0/(-2*r*sin(0.5*dtheta)))
            theta0 = (asin(theta0) - dtheta*(t-0.5))    # asin only covers -pi/2 to pi/2
            # Problem: Quadrant
            
            # theta0 = measurement[1] - pre[1]
            # theta0 = (theta0/(2*r*sin(0.5*dtheta)))
            # theta0 = (acos(theta0) - dtheta*(t-0.5)) % (2*pi)
            print("theta0 = ", theta0)

            xy_estimate = [measurement[0] - r*2*sin(theta0 + dtheta*(t+0.5))*sin(0.5*dtheta), \
                        measurement[1] + r*2*cos(theta0 + dtheta*(t+0.5))*sin(0.5*dtheta)]
        else:
            xy_estimate = measurement
        vector_pre = vector
    pre = measurement   
    
    OTHER = [t, pre, vector_pre, r_ls, dtheta_ls, theta0_ls]
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




