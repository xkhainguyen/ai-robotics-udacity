# LECTURE 23: PROJECT RUNAWAY ROBOT, PART 2: ADDING NOISE
# By Xuan Khai Nguyen
#####################
# This program uses Least-Squares Circle Fit* to estimate the distance and turning angle of the robot,
# hence, predict the next position. The approach calculates the distance of two points and 
# the turning angle by the difference between two distance vector. Therefore it requires at 
# least three points noiseless case and average running windows in noise case.
#####################
# Refer to L23_P1_LS_Khai.py for more introductory details
# More average, better result

from numpy import mean
from robot import *  # Check the robot.py tab to see how this works.
from math import *
from matrix import * # Check the matrix.py tab to see how this works.
t = 0
k = 0
def estimate_next_pos(measurement, OTHER = None):
    x,y = measurement
    global t, k
    if OTHER:
        t += 1

        OTHER.append(measurement)
        if len(OTHER) > W_SIZE:
            OTHER.pop(0)
            k += 1

        xbar = mean([x for x, __ in OTHER]) #average x coords
        ybar = mean([y for __, y in OTHER]) #average y coords

        #parameters for finding the center of the circle
        ui = [x - xbar for x, __ in OTHER] 
        vi = [y - ybar for __, y in OTHER]

        #partial derivatives of center finding function that we want to minimize
        Suu = sum(u**2 for u in ui)
        Suuu = sum(u**3 for u in ui)
        Suv = sum(u*v for u, v in zip(ui, vi))
        Suvv = sum(u*v**2 for u, v in zip(ui, vi))
        Svuu = sum(v*u**2 for u, v in zip(ui, vi))
        Svv = sum(v**2 for v in vi)
        Svvv = sum(v**3 for v in vi)

        #print matrix([[Suu, Suv],[Suv, Svv]])
        try:
            res = (matrix([[Suu, Suv],[Suv, Svv]]).inverse() *\
                    matrix([[.5*(Suuu + Suvv)],[.5*(Svvv + Svuu)]])).value

        #cannot do circular regression on only 2 points, inverse will give ZeroDivision
        except ZeroDivisionError: 
            return measurement, OTHER

        uc, vc = (res[0][0], res[1][0])
        radius = sqrt(uc**2 + vc**2 + (Suu + Svv)/float(len(OTHER)))
        xc, yc = (uc + xbar, vc + ybar)

        # calculate delta theta ~ turning angle
        del_theta_l = [(atan2(y2-yc, x2-xc) - atan2(y1-yc, x1-xc) + 2*pi)%(2*pi)
                            for ((x1, y1), (x2, y2)) in zip(OTHER[:-1], OTHER[1:])]
        mean_del_theta = mean(del_theta_l)
        # print("dtheta = ", mean_del_theta)

        # calculate theta0 ~ initial heading angle       
        theta = [atan2(y - yc, x - xc) for x, y in OTHER]                   
        theta0_l = [(thetai - i*mean_del_theta)%(2*pi) for thetai, i in zip(theta,list(range(k,t+1)))]
        # print(theta_l)
        theta0 = mean(theta0_l)
        # print("theta0 = ", theta0)

        # estimate next position
        xy_estimate = (radius*cos(theta0+(t+1)*mean_del_theta) + xc, radius*sin(theta0+(t+1)*mean_del_theta) + yc)

    else:
        xy_estimate = measurement
        OTHER = [measurement]

    return xy_estimate, OTHER

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
            localized = True
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
measurement_noise = 0.05 * test_target.distance
test_target.set_noise(0.0, 0.0, measurement_noise)
W_SIZE = 50
MAX_CTR = 1000
demo_grading(estimate_next_pos, test_target)