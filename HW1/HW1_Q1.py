#!/usr/bin/env python
# -*- coding: utf-8 -*-
#HW1 for RBE 595/CS 525 Motion Planning
#code based on the simplemanipulation.py example
import time
import openravepy

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def tuckarms(env,robot):
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

if __name__ == "__main__":

	env = Environment()
	env.SetViewer('qtcoin')
	env.Reset()
	# load a scene from ProjectRoom environment XML file
	env.Load('data/pr2test2.env.xml')
	time.sleep(0.1)

	# 1) get the 1st robot that is inside the loaded scene
	# 2) assign it to the variable named 'robot'
	robot = env.GetRobots()[0]

	# tuck in the PR2's arms for driving
	tuckarms(env,robot);

	#### YOUR CODE HERE ####
	table6 = env.GetBodies()[6]
	table6.SetTransform(numpy.array([[ 1.        ,  0.        ,  0.        ,  2.27124786],
	[ 0.        ,  1.        ,  0.        , -1.17899954],
	[ 0.        ,  0.        ,  1.        ,  0.73999774],
	[ 0.        ,  0.        ,  0.        ,  1.        ]]))

	table3 = env.GetBodies()[3]
	table3.SetTransform(numpy.array([[ 0.        , -1.        ,  0.        , -0.98851043],
       [ 1.        ,  0.        ,  0.        , -1.48659539],
       [ 0.        ,  0.        ,  1.        ,  0.7400074 ],
       [ 0.        ,  0.        ,  0.        ,  1.        ]]))


	table1 = env.GetBodies()[1]
	table1.SetTransform(numpy.array([[ 0.        , -1.        ,  0.        , -2.1942718 ],
       [ 1.        ,  0.        ,  0.        , -1.4904089 ],
       [ 0.        ,  0.        ,  1.        ,  0.74001443],
       [ 0.        ,  0.        ,  0.        ,  1.        ]]))
	table2 = env.GetBodies()[2]
	table2.SetTransform(numpy.array([[ -2.22044605e-16,  -1.00000000e+00,   0.00000000e+00,
         -2.19190598e+00],
       [  1.00000000e+00,  -2.22044605e-16,   0.00000000e+00,
         -8.88029520e-01],
       [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00,
          7.40010260e-01],
       [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
          1.00000000e+00]]))

	table4 = env.GetBodies()[4]
	table4.SetTransform(numpy.array([[ 0.        , -1.        ,  0.        , -0.98906225],
       [ 1.        ,  0.        ,  0.        , -0.88476783],
       [ 0.        ,  0.        ,  1.        ,  0.7400167 ],
       [ 0.        ,  0.        ,  0.        ,  1.        ]]))


	#### END OF YOUR CODE ###


	raw_input("Press enter to exit...")
