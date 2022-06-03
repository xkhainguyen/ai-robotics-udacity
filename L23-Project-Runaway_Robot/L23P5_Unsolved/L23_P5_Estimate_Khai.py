"""Reference from https://github.com/jeremy-shannon/udacity-AI-for-robotics/blob/master/Project%20-%20Runaway%20Robot/studentMain.py
Constructing the EKF:
  X = [x, y, distance, theta, turning]
  System: 
    X(k+1) = F(x(k))*x(k) + u; 
    z(k+1) = H(x(k+1))*x(k+1)
  Linearizing: 
    del_X(k+1) = A(x(k))*del_X(k); 
    del_z(k+1) = B(x(k+1))*x(k+1);
    with A is Jacobian of F, B is Jacobian of H 
  
  Warm start: 
    Feed initial variables using calculations via 3 points
  Repeat until convergence:
    Step 1: Update
    Step 2: Predict
"""

from robot import *  # Check the robot.py tab to see how this works.
from math import *
from matrix import * # Check the matrix.py tab to see how this works.
import random

######################### CARTESIAN ###################################
def estimate_next_pos(measurement, OTHER = None):
    x1 = measurement[0]
    y1 = measurement[1]

    try:
        cntr = OTHER[2]
    except:
        pass

    if not OTHER:
        cntr = 1
        x = matrix([[x1],[y1],[0],[0],[0]])          
        OTHER = [x, [], cntr]
        xy_estimate = [x1, y1]

    elif cntr == 1:
        cntr += 1 
        # Warm start at the first step
        x0 = OTHER[0].value[0][0]
        y0 = OTHER[0].value[1][0]
        dist0 = distance_between([x0, y0], [x1, y1])
        theta0 = atan2(y1 - y0, x1 - x0)
        dtheta0 = 0.

        x = matrix([[x1],[y1],[dist0],[theta0],[dtheta0]])                    
        OTHER = [x, [], cntr]
        xy_estimate = [x1, y1]

    elif cntr == 2:
        cntr += 1
        # Warm start at the second step
        x0 = OTHER[0].value[0][0]
        y0 = OTHER[0].value[1][0]
        dist0 = OTHER[0].value[2][0]
        dist0 = (dist0 + distance_between([x0, y0], [x1, y1]))/2
        theta0 = OTHER[0].value[3][0]
        dtheta0 = (atan2(y1 - y0, x1 - x0) - theta0) % (2*pi)
        theta0 = atan2(y1 - y0, x1 - x0)
        # initial uncertainty: 
        P =  matrix([[100.,0.,0.,0.,0.],
                     [0.,100.,0.,0.,0.],
                     [0.,0.,100.,0.,0.],
                     [0.,0.,0.,100.,0.],
                     [0.,0.,0.,0,100.]])     

        x = matrix([[x1],[y1],[dist0],[theta0],[dtheta0]])                      
        OTHER = [x, P, cntr]
        xy_estimate = [x1, y1]
    # END OF WARM START, REPEAT UNTIL CONVERGENCE
    else:
        # pull previous measurement, state variables (x), and uncertainty (P) from OTHER
        x0 = OTHER[0].value[0][0]
        y0 = OTHER[0].value[1][0]
        dist0 = OTHER[0].value[2][0]
        theta0 = OTHER[0].value[3][0] % (2*pi)
        dtheta0 = OTHER[0].value[4][0]
        P = OTHER[1]
  
        # time step
        dt = 1.
            
        # state matrix (polar location and angular velocity)
        x = matrix([[x0],[y0],[dist0],[theta0],[dtheta0]]) 
        # external motion
        u = matrix([[0.], [0.], [0.], [0.], [0.]]) 

        # measurement function: 
        # for the EKF this should be the Jacobian of H, but in this case it turns out to be the same
        H =  matrix([[1.,0.,0.,0.,0.],
                    [0.,1.,0.,0.,0.]])
        # measurement uncertainty: 
        R =  matrix([[0.7,0.],
                    [0.,0.7]])
        # 5d identity matrix
        I =  matrix([[]])
        I.identity(5)

        
        # measurement update
        Z = matrix([[x1,y1]])
        y = Z.transpose() - (H * x)
        S = H * P * H.transpose() + R
        K = P * H.transpose() * S.inverse()
        x = x + (K * y)
        P = (I - (K * H)) * P
        
        # pull out current estimates based on measurement
        # this was a big part of what was hainging me up (I was using older estimates before)
        x0 = x.value[0][0]
        y0 = x.value[1][0]
        dist0 = x.value[2][0]
        theta0 = x.value[3][0] % (2*pi)
        dtheta0 = x.value[4][0]

        # next state function: 
        # this is now the Jacobian of the transition matrix (F) from the regular Kalman Filter
        A =  matrix([[1.,0.,cos(theta0+dtheta0),-dist0*sin(theta0+dtheta0),-dist0*sin(theta0+dtheta0)],
                    [0.,1.,sin(theta0+dtheta0),dist0*cos(theta0+dtheta0),dist0*cos(theta0+dtheta0)],
                    [0.,0.,1.,0.,0.],
                    [0.,0.,0.,1.,dt],
                    [0.,0.,0.,0.,1.]])

        # Predict
        x = matrix([[x0 + dist0 * cos(theta0 + dtheta0)],
                    [y0 + dist0 * sin(theta0 + dtheta0)],
                    [dist0],
                    [(theta0 + dtheta0) % (2*pi)],
                    [dtheta0]])

        P = A * P * A.transpose()

        OTHER[0] = x
        OTHER[1] = P
        
        xy_estimate = (x.value[0][0], x.value[1][0])
        
    return xy_estimate, OTHER

# A helper function you may find useful.
def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def demo_grading(estimate_next_pos_fcn, target_bot, OTHER = None):
    localized = False
    distance_tolerance = 0.02 * target_bot.distance
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
def demo_grading2(estimate_next_pos_fcn, target_bot, OTHER = None):
    localized = False
    distance_tolerance = 0.02 * target_bot.distance
    ctr = 0
    # if you haven't localized the target bot, make a guess about the next
    # position, then we move the bot and compare your guess to the true
    # next position. When you are close enough, we stop checking.
    while not localized and ctr <= 1000:
        ctr += 1
        measurement = target_bot.sense()
        position_guess, OTHER = estimate_next_pos_fcn(measurement, OTHER)
        target_bot.move_in_circle()
        true_position = (target_bot.x, target_bot.y)
        error = distance_between(position_guess, true_position)
        if error <= distance_tolerance:
            print ("You got it right! It took you ", ctr, " steps to localize.")
            localized = True
        if ctr == 1000:
            print ("Sorry, it took you too many steps to localize the target.")
    return localized
# MAIN 
test_target = robot(0., 10., 0.0, 2*pi / 30.0, 1.5)
measurement_noise = 2.0 * test_target.distance
test_target.set_noise(0.0, 0.0, measurement_noise)
MAX_CTR = 1000
demo_grading2(estimate_next_pos, test_target)



