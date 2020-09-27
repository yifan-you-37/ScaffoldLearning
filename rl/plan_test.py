
import pybullet as p
import time
import numpy as np
import sys
import os

simulation_dir = '../simulation'
sys.path.insert(0, simulation_dir)

from Two_Arm_Manipulation_Env import RobotEnv
import robot_no_Wrench as robot

p_id = p.connect(p.GUI)

env = RobotEnv(0, p)
env.reset()

for i in range(1000):
    time.sleep(0.01)
    p.stepSimulation()
# def isStateValid(state):
#     # "state" is of type SE2StateInternal, so we don't need to use the "()"
#     # operator.
#     #
#     # Some arbitrary condition on the state (note that thanks to
#     # dynamic type checking we can just call getX() and do not need
#     # to convert state to an SE2State.)
#     return state.getX() < .6

 
# def plan():
#     space = ob.CompoundStateSpace()
#     robot_subspace = ob.RealVectorStateSpace()
#     for (one_ll, one_ul) in zip(env.robot.ll, env.robot.ul):
#         robot_subspace.addDimension(one_ll, one_ul)
#     obj_se3_subspace = ob.SE3StateSpace()
    

#     # set lower and upper bounds
#     bounds = ob.RealVectorBounds(2)
#     bounds.setLow(-1)
#     bounds.setHigh(1)
#     space.setBounds(bounds)
 
#     # create a simple setup object
#     ss = og.SimpleSetup(space)
#     ss.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))
 
#     start = ob.State(space)
#     # we can pick a random start state...
#     start.random()
#     # ... or set specific values
#     start().setX(.5)
 
#     goal = ob.State(space)
#     # we can pick a random goal state...
#     goal.random()
#     # ... or set specific values
#     goal().setX(-.5)
 
#     ss.setStartAndGoalStates(start, goal)
 
#     # this will automatically choose a default planner with
#     # default parameters
#     solved = ss.solve(1.0)
 
#     if solved:
#         # try to shorten the path
#         ss.simplifySolution()
#         # print the simplified path
#         print (ss.getSolutionPath())
 
 
# if __name__ == "__main__":
#     plan()
