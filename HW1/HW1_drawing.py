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

    # red rectangles for table

    handles = []
    for i in range (len(env.GetBodies())):
        if(i>0 and i<5):
            front_table = env.GetBodies()[i];
            table_centre = front_table.GetTransform()[:3,3]
            handles.append(env.drawlinestrip(points=array(((table_centre[0] - 0.6,table_centre[1]-0.3,0.74),(table_centre[0]-0.6,table_centre[1] + 0.3,0.74),(table_centre[0]+0.6,table_centre[1] + 0.3,0.74),(table_centre[0]+0.6,table_centre[1]-0.3,0.74),
                                            (table_centre[0]-0.6,table_centre[1]-0.3,0.74))),
                                           linewidth=5.0,
                                           colors=array(((1,0,0),(1,0,0),(1,0,0),(1,0,0),(1,0,0)))))
        if(i == 5 or i == 6):
            back_table = env.GetBodies()[i];
            table_centre = back_table.GetTransform()[:3,3]
            handles.append(env.drawlinestrip(points=array(((table_centre[0] - 0.3,table_centre[1]-0.6,0.74),(table_centre[0]-0.3,table_centre[1] + 0.6,0.74),
                                           (table_centre[0]+0.3,table_centre[1] + 0.6,0.74),(table_centre[0]+0.3,table_centre[1] - 0.6,0.74),(table_centre[0]-0.3,table_centre[1] - 0.6,0.74))),
                                           linewidth=5.0,
                                           colors=array(((1,0,0),(1,0,0),(1,0,0),(1,0,0),(1,0,0))))) 
    # blue circle around the environment

    radius = math.sqrt(22)
    theta = 0
    for i in range(35):

        handles.append(env.plot3(points=array((radius*math.cos(math.radians(theta)),radius*math.sin(math.radians(theta)),0)),
                            pointsize=0.1,
                            colors=array(((0,0,1))),
                            drawstyle=1))
        theta = theta + 10.285


    
    #### END OF YOUR CODE ###


    raw_input('Enter any key to quit. ')
                                   



