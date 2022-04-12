# -------------------
# Background Information
#
# In this problem, you will build a planner that helps a robot
# find the shortest way in a warehouse filled with boxes
# that he has to pick up and deliver to a drop zone.
# 
# For example:
#
# warehouse = [[ 1, 2, 3],
#              [ 0, 0, 0],
#              [ 0, 0, 0]]
# dropzone = [2,0] 
# todo = [2, 1]
# 
# The robot starts at the dropzone.
# The dropzone can be in any free corner of the warehouse map.
# todo is a list of boxes to be picked up and delivered to the dropzone.
#
# Robot can move diagonally, but the cost of a diagonal move is 1.5.
# The cost of moving one step horizontally or vertically is 1.
# So if the dropzone is at [2, 0], the cost to deliver box number 2
# would be 5.

# To pick up a box, the robot has to move into the same cell as the box.
# When the robot picks up a box, that cell becomes passable (marked 0)
# The robot can pick up only one box at a time and once picked up 
# it has to return the box to the dropzone by moving onto the dropzone cell.
# Once the robot has stepped on the dropzone, the box is taken away, 
# and it is free to continue with its todo list.
# Tasks must be executed in the order that they are given in the todo list.
# You may assume that in all warehouse maps, all boxes are
# reachable from beginning (the robot is not boxed in).

# -------------------
# User Instructions
#
# Design a planner (any kind you like, so long as it works!)
# in a function named plan() that takes as input three parameters: 
# warehouse, dropzone, and todo. See parameter info below.
#
# Your function should RETURN the final, accumulated cost to do
# all tasks in the todo list in the given order, which should
# match with our answer. You may include print(statements to show 
# the optimum path, but that will have no effect on grading.
#
# Your solution must work for a variety of warehouse layouts and
# any length of todo list.
# 
# Add your code at line 76.
# 
# --------------------
# Parameter Info
#
# warehouse - a grid of values, where 0 means that the cell is passable,
# and a number 1 <= n <= 99 means that box n is located at that cell.
# dropzone - determines the robot's start location and the place to return boxes 
# todo - list of tasks, containing box numbers that have to be picked up
#
# --------------------
# Testing
#
# You may use our test function below, solution_check(),
# to test your code for a variety of input parameters. 

from math import *

warehouse = [[ 1, 2, 3],
             [ 0, 0, 0],
             [ 0, 0, 0]]
dropzone = [2,0] 
todo = [2, 1]
# ------------------------------------------
# plan - Returns cost to take all boxes in the todo list to dropzone
#
# ----------------------------------------
# modify code below
# ----------------------------------------

def make_heuristic(grid, goal, cost):
    heuristic = [[0 for row in range(len(grid[0]))] 
                      for col in range(len(grid))]
    for i in range(len(grid)):    
        for j in range(len(grid[0])):
            heuristic[i][j] = sqrt(abs(i - goal[0]) + abs(j - goal[1]))
    return heuristic

def A_star_search(grid,init,goal,delta,delta_name, cost,heuristic):
    # ----------------------------------------
    # modify the code below
    # ----------------------------------------
    closed = [[0 for col in range(len(grid[0]))] for row in range(len(grid))]
    closed[init[0]][init[1]] = 1

    expand = [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]
    action = [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]

    x = init[0]
    y = init[1]
    g = 0
    h = heuristic[x][y]
    f = g + h

    open = [[f, g, h, x, y]]

    found = False  # flag that is set when search is complete
    resign = False # flag set if we can't find expand
    count = 0
    
    while not found and not resign:
        if len(open) == 0:
            resign = True
            return("Fail")
        else:
            open.sort()
            open.reverse()
            next = open.pop()
            x = next[3]
            y = next[4]
            g = next[1]
            expand[x][y] = count
            count += 1
            
            if x == goal[0] and y == goal[1]:
                found = True
                cost_all = g
            else:
                for i in range(len(delta)):
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    if x2 >= 0 and x2 < len(grid) and y2 >=0 and y2 < len(grid[0]):
                        if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                            g2 = g + cost[i]
                            h2 = heuristic[x2][y2]
                            f2  = g2 + h2
                            open.append([f2, g2, h2, x2, y2])
                            closed[x2][y2] = 1
                            action[x2][y2] = i

    for c in action:
        print(c)
    print()

    policy = [[' ' for col in range(len(grid[0]))] for row in range(len(grid))]
    x = goal[0]
    y = goal[1]
    policy[x][y] = '*'
    while x!=init[0] or y!=init[1]:
        x2 = x - delta[action[x][y]][0]
        y2 = y - delta[action[x][y]][1]
        policy[x2][y2] = delta_name[action[x][y]]
        x = x2
        y = y2
    for c in policy:
        print(c)
    print()      

    return cost_all

def plan(warehouse, dropzone, todo):

    delta = [[-1,  0], # go up0
             [-1, -1],
             [ 0, -1], # go left2
             [ 1, -1],
             [ 1,  0], # go down4
             [ 1,  1],
             [ 0,  1], # go right6
             [-1,  1]]

    delta_name = ['^', '\\', '<', '/', 'v', '\\', '>', '/']          

    cost_delta  = [1, 1.5, 1, 1.5, 1, 1.5, 1, 1.5]
    cost        = 0.

    grid = [[warehouse[row][col] for col in range(len(warehouse[0]))] for row in range(len(warehouse))]

    for i in range(len(todo)):

        # go to pick up a box
        init = dropzone

        for col in range(len(grid[0])):
            for row in range(len(grid)):
                if grid[row][col] == todo[i]:
                    goal = [row, col]
                    grid[row][col] = 0

        heuristic = make_heuristic(grid, goal, cost_delta)    
        a = A_star_search(grid, init, goal, delta, delta_name, cost_delta, heuristic)
        if a != "Fail":
            cost += a
        else:
            return("Fail")
        
        # go to dropzone
        init = goal
        goal = dropzone

        heuristic = make_heuristic(grid, goal, cost_delta)    

        a = A_star_search(grid, init, goal, delta, delta_name, cost_delta, heuristic)
        if a != "Fail":
            cost += a
        else:
            return("Fail")
    return cost
    
################# TESTING ##################
       
# ------------------------------------------
# solution check - Checks your plan function using
# data from list called test[]. Uncomment the call
# to solution_check to test your code.
#
def solution_check(test, epsilon = 0.00001):
    answer_list = []
    
    import time
    start = time.time()
    correct_answers = 0
    for i in range(len(test[0])):
        user_cost = plan(test[0][i], test[1][i], test[2][i])
        true_cost = test[3][i]
        if abs(user_cost - true_cost) < epsilon:
            print("\nTest case", i+1, "passed!")
            answer_list.append(1)
            correct_answers += 1
            #print("#############################################"
        else:
            print("\nTest case ", i+1, "unsuccessful. Your answer ", user_cost, "was not within ", epsilon, "of ", true_cost) 
            answer_list.append(0)
    runtime =  time.time() - start
    if runtime > 1:
        print("Your code is too slow, try to optimize it! Running time was: ", runtime)
        return False
    if correct_answers == len(answer_list):
        print("\nYou passed all test cases!")
        return True
    else:
        print("\nYou passed", correct_answers, "of", len(answer_list), "test cases. Try to get them all!")
        return False
#Testing environment
# Test Case 1 
warehouse1 = [[ 1, 2, 3],
              [ 0, 0, 0],
              [ 0, 0, 0]]
dropzone1 = [2,0] 
todo1 = [2, 1]
true_cost1 = 9
# Test Case 2
warehouse2 = [[ 1, 2, 3, 4],
              [ 0, 0, 0, 0],
              [ 5, 6, 7, 0],
              [ 0, 0, 0, 8]] 
dropzone2 = [3,0] 
todo2 = [2, 5, 1]
true_cost2 = 21

# Test Case 3
warehouse3 = [[ 1, 2,  3,  4, 5, 6,  7],
              [ 0, 0,  0,  0, 0, 0,  0],
              [ 8, 9, 10, 11, 0, 0,  0],
              [ 0, 0,  0,  0, 0, 0, 12]] 
dropzone3 = [3,0] 
todo3 = [5, 10]
true_cost3 = 18

# Test Case 4
warehouse4 = [[ 1, 17, 5, 18,  9, 19,  13],
              [ 2,  0, 6,  0, 10,  0,  14],
              [ 3,  0, 7,  0, 11,  0,  15],
              [ 4,  0, 8,  0, 12,  0,  16],
              [ 0,  0, 0,  0,  0,  0,   0]] 
dropzone4 = [4,6]
todo4 = [13, 11, 6, 17]
true_cost4 = 41

testing_suite = [[warehouse4],
                 [dropzone4],
                 [todo4],
                 [true_cost4]]


solution_check(testing_suite) #UNCOMMENT THIS LINE TO TEST YOUR CODE
