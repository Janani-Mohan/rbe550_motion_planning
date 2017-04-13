#!/usr/bin/env python
# -*- coding: utf-8 -*-
#HW1 for RBE 595/CS 525 Motion Planning
#code based on the simplemanipulation.py example
import time
import openravepy

#### YOUR IMPORTS GO HERE ####
import heapq
import numpy as np
import math
#### END OF YOUR IMPORTS ####

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

class Robot_Pose(object):

    def __init__(self, values):
        self.x = values[0]
        self.y = values[1]
        self.o = values[2]

class State(object):

    def __init__(self, Robot_Pose):
        self.Pose = str([Robot_Pose.x, Robot_Pose.y, Robot_Pose.o])
        self.point = Robot_Pose
        self.parent = None
        self.g_score = 0
        self.h_score = 0

    def __eq__(self, other):
        if  self.point.x == other.point.x and self.point.y == other.point.y and self.point.o == other.point.o:
            other.g_score = self.g_score
        return self.point.x == other.point.x and self.point.y == other.point.y and self.point.o == other.point.o

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return hash(str(self.Pose))

class Astar(object):

    def __init__(self):

        self.openSet = []
        self.closedSet = []

    # function to calculate the Manhattan distance between two points
    def Manhattan(self, point1, point2):
        return math.fabs(point1.x - point2.x) + math.fabs(point1.y - point2.y) + math.fabs(point1.o - point2.o)

    # function to calculate the Euclidean distance between two points
    def Euclidean(self, point1,point2):
        return math.sqrt(math.pow((point1.x - point2.x),2) + math.pow((point1.y - point2.y),2) + math.pow((point1.o - point2.o),2))

    # function to check collision of the robot with environment
    def checkCollision(self, neighbour, env, robot):
        robot.SetActiveDOFValues([neighbour.point.x, neighbour.point.y, neighbour.point.o])
        inCollision = env.CheckCollision(robot)
        return inCollision

    # function to draw the explored nodes and nodes in collision
    def Plot_Points(self, neighbour, inCollision, handles):
        if inCollision:
            handles.append(env.plot3(points=array((( neighbour.point.x, neighbour.point.y , 0 ))),pointsize=0.03,colors=array(((1,0,0))),drawstyle=1))
        else:
            handles.append(env.plot3(points=array((( neighbour.point.x, neighbour.point.y , 0 ))),pointsize=0.03,colors=array(((0,0,1))),drawstyle=1))

    # function to find the successors of the current node
    def find_successors(self, current_state, n_connected, stepsizeX, stepsizeY, stepsizeO, env, robot, handles):

        successor_set = set()

        n_x = current_state.point.x
        n_y = current_state.point.y
        n_z = current_state.point.o

        neighbour = State(Robot_Pose([n_x + stepsizeX, n_y, n_z ]))
        inCollision = self.checkCollision(neighbour, env, robot)
        self.Plot_Points(neighbour, inCollision, handles)
        successor_set.add(neighbour)

        neighbour = State(Robot_Pose([n_x, n_y + stepsizeY, n_z]))
        inCollision = self.checkCollision(neighbour, env, robot)
        self.Plot_Points(neighbour, inCollision, handles)
        successor_set.add(neighbour)

        neighbour = State(Robot_Pose([n_x - stepsizeX, n_y, n_z]))
        inCollision = self.checkCollision(neighbour, env, robot)
        self.Plot_Points(neighbour, inCollision, handles)
        successor_set.add(neighbour)

        neighbour = State(Robot_Pose([n_x, n_y - stepsizeY, n_z]))
        inCollision = self.checkCollision(neighbour, env, robot)
        self.Plot_Points(neighbour, inCollision, handles)
        successor_set.add(neighbour)

        neighbour = State(Robot_Pose([n_x, n_y, n_z + stepsizeO]))
        inCollision = self.checkCollision(neighbour, env, robot)
        self.Plot_Points(neighbour, inCollision, handles)
        successor_set.add(neighbour)

        neighbour = State(Robot_Pose([n_x, n_y , n_z - stepsizeO]))
        inCollision = self.checkCollision(neighbour, env, robot)
        self.Plot_Points(neighbour, inCollision, handles)
        successor_set.add(neighbour)

        if n_connected == 8:

            neighbour = State(Robot_Pose([n_x + stepsizeX, n_y, n_z + stepsizeO ]))
            inCollision = self.checkCollision(neighbour, env, robot)
            self.Plot_Points(neighbour, inCollision, handles)
            successor_set.add(neighbour)

            neighbour = State(Robot_Pose([n_x + stepsizeX, n_y, n_z - stepsizeO ]))
            inCollision = self.checkCollision(neighbour, env, robot)
            self.Plot_Points(neighbour, inCollision, handles)
            successor_set.add(neighbour)

            neighbour = State(Robot_Pose([n_x, n_y + stepsizeY, n_z + stepsizeO]))
            inCollision = self.checkCollision(neighbour, env, robot)
            self.Plot_Points(neighbour, inCollision, handles)
            successor_set.add(neighbour)

            neighbour = State(Robot_Pose([n_x, n_y + stepsizeY, n_z - stepsizeO]))
            inCollision = self.checkCollision(neighbour, env, robot)
            self.Plot_Points(neighbour, inCollision, handles)
            successor_set.add(neighbour)

            neighbour = State(Robot_Pose([n_x - stepsizeX, n_y, n_z + stepsizeO]))
            inCollision = self.checkCollision(neighbour, env, robot)
            self.Plot_Points(neighbour, inCollision, handles)
            successor_set.add(neighbour)

            neighbour = State(Robot_Pose([n_x - stepsizeX, n_y, n_z - stepsizeO]))
            inCollision = self.checkCollision(neighbour, env, robot)
            self.Plot_Points(neighbour, inCollision, handles)
            successor_set.add(neighbour)

            neighbour = State(Robot_Pose([n_x, n_y - stepsizeY, n_z + stepsizeO]))
            inCollision = self.checkCollision(neighbour, env, robot)
            self.Plot_Points(neighbour, inCollision, handles)
            successor_set.add(neighbour)


            neighbour = State(Robot_Pose([n_x, n_y - stepsizeY, n_z - stepsizeO]))
            inCollision = self.checkCollision(neighbour, env, robot)
            self.Plot_Points(neighbour, inCollision, handles)
            successor_set.add(neighbour)

            neighbour = State(Robot_Pose([n_x + stepsizeX, n_y + stepsizeY, n_z ]))
            inCollision = self.checkCollision(neighbour, env, robot)
            self.Plot_Points(neighbour, inCollision, handles)
            successor_set.add(neighbour)

            neighbour = State(Robot_Pose([n_x - stepsizeX, n_y + stepsizeY, n_z]))
            inCollision = self.checkCollision(neighbour, env, robot)
            self.Plot_Points(neighbour, inCollision, handles)
            successor_set.add(neighbour)

            neighbour = State(Robot_Pose([n_x + stepsizeX, n_y - stepsizeY, n_z]))
            inCollision = self.checkCollision(neighbour, env, robot)
            self.Plot_Points(neighbour, inCollision, handles)
            successor_set.add(neighbour)

            neighbour = State(Robot_Pose([n_x - stepsizeX, n_y - stepsizeY, n_z]))
            inCollision = self.checkCollision(neighbour, env, robot)
            self.Plot_Points(neighbour, inCollision, handles)
            successor_set.add(neighbour)

            neighbour = State(Robot_Pose([n_x + stepsizeX, n_y + stepsizeY, n_z + stepsizeO]))
            inCollision = self.checkCollision(neighbour, env, robot)
            self.Plot_Points(neighbour, inCollision, handles)
            successor_set.add(neighbour)

            neighbour = State(Robot_Pose([n_x + stepsizeX, n_y + stepsizeY , n_z - stepsizeO]))
            inCollision = self.checkCollision(neighbour, env, robot)
            self.Plot_Points(neighbour, inCollision, handles)
            successor_set.add(neighbour) 

            neighbour = State(Robot_Pose([n_x + stepsizeX, n_y - stepsizeY, n_z + stepsizeO]))
            inCollision = self.checkCollision(neighbour, env, robot)
            self.Plot_Points(neighbour, inCollision, handles)
            successor_set.add(neighbour)

            neighbour = State(Robot_Pose([n_x + stepsizeX, n_y - stepsizeY , n_z - stepsizeO]))
            inCollision = self.checkCollision(neighbour, env, robot)
            self.Plot_Points(neighbour, inCollision, handles)
            successor_set.add(neighbour)  

            neighbour = State(Robot_Pose([n_x - stepsizeX, n_y + stepsizeY, n_z + stepsizeO]))
            inCollision = self.checkCollision(neighbour, env, robot)
            self.Plot_Points(neighbour, inCollision, handles)
            successor_set.add(neighbour)

            neighbour = State(Robot_Pose([n_x - stepsizeX, n_y + stepsizeY , n_z - stepsizeO]))
            inCollision = self.checkCollision(neighbour, env, robot)
            self.Plot_Points(neighbour, inCollision, handles)
            successor_set.add(neighbour)

            neighbour = State(Robot_Pose([n_x - stepsizeX, n_y - stepsizeY, n_z + stepsizeO]))
            inCollision = self.checkCollision(neighbour, env, robot)
            self.Plot_Points(neighbour, inCollision, handles)
            successor_set.add(neighbour)

            neighbour = State(Robot_Pose([n_x - stepsizeX, n_y - stepsizeY , n_z - stepsizeO]))
            inCollision = self.checkCollision(neighbour, env, robot)
            self.Plot_Points(neighbour, inCollision, handles)
            successor_set.add(neighbour)        

        return successor_set
    # function which returns True if the state exists in the Closed set
    def states_in_ClosedSet(self, neighbour, closedSet):
        if neighbour in closedSet:
            return True
        else:
            return False
    # function which returns True if the state exists in the Open set
    def states_in_OpenSet(self, neighbour, openSet):
        if neighbour in openSet:
            return True
        else:
            return False
    # function to find the cost g(n)
    def find_cost(self, current_state, neighbour):
        distance = self.Euclidean(current_state.point, neighbour.point)
        return distance
    # function to check if the goal has been reached
    def isGoal(self, current_state, goal_state , stepsizeX , stepsizeY , stepsizeO):
        if abs(current_state.point.x - goal_state.point.x) <= stepsizeX/2  and abs(current_state.point.y - goal_state.point.y) <= stepsizeY/2  and abs(current_state.point.o - goal_state.point.o) <= stepsizeO:
            print " Reached Goal  == X: %f Y: %f o: %f" % (current_state.point.x, current_state.point.y, current_state.point.o)
            return True
        else:
            return False
    # function which implements the main algorithm
    def algorithm(self,startconfig, goal, n_connected, heuristic, env, robot):

        start_state = State(Robot_Pose(startconfig))
        goal_state = State(Robot_Pose(goal))

        start_state.g_score = 0
        if heuristic == "Manhattan":
            start_state.h_score = self.Manhattan(start_state.point, goal_state.point)
        else:
            start_state.h_score = self.Euclidean(start_state.point, goal_state.point)            
        stepsizeX = 0.1
        stepsizeY = 0.1
        stepsizeO = np.pi/4;

        current_state = start_state
        self.openSet.append(current_state)

        while len(self.openSet) > 0:

            current_state = min(self.openSet,key=lambda o:o.h_score + o.g_score)
            
            self.openSet.remove(current_state)
            self.closedSet.append(current_state)

            if self.isGoal(current_state, goal_state , stepsizeX , stepsizeY , stepsizeO):
                path = []
                while current_state.parent:
                    path.append((current_state))
                    current_state = current_state.parent
                path.append(current_state)
                return path[::-1]

            successor_set = self.find_successors(current_state, n_connected, stepsizeX , stepsizeY , stepsizeO , env, robot, handles)

            for neighbour in successor_set:

                collisionCheck = self.checkCollision(neighbour, env, robot)

                if self.states_in_ClosedSet(neighbour, self.closedSet) or collisionCheck:
                    continue

                if len(self.openSet) == 0:
                    neighbour.g_score = current_state.g_score + self.find_cost(current_state, neighbour)
                    neighbour.parent = current_state
                    if heuristic == "Manhattan":
                        neighbour.h_score = self.Manhattan(neighbour.point, goal_state.point)
                    else:
                        neighbour.h_score = self.Euclidean(neighbour.point, goal_state.point)
                    self.openSet.append((neighbour))

                else:
                    if self.states_in_OpenSet(neighbour, self.openSet):
                        temp_g = current_state.g_score + self.find_cost(current_state, neighbour)
                        if neighbour.g_score > temp_g:
                            neighbour.g_score = temp_g
                            neighbour.parent = current_state
                    else:
                        neighbour.g_score = current_state.g_score + self.find_cost(current_state, neighbour)
                        if heuristic == "Manhattan":
                            neighbour.h_score = self.Manhattan(neighbour.point, goal_state.point)
                        else:
                            neighbour.h_score = self.Euclidean(neighbour.point, goal_state.point)
                        neighbour.parent = current_state
                        self.openSet.append((neighbour))
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

    with env:
        # the active DOF are translation in X and Y and rotation about the o axis of the base of the robot.
        robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])

        goalconfig = [2.6,-1.3,-pi/2]
        #### YOUR CODE HERE ####
        handles = []
        n_connected = 8;
        heuristic = "Euclidean"
        #### Implement the A* algorithm to compute a path for the robot's base starting from the current configuration of the robot and ending at goalconfig. The robot's base DOF have already been set as active. It may be easier to implement this as a function in a separate file and call it here.
        solver = Astar()        
        startconfig = robot.GetActiveDOFValues()
        goal = np.array([goalconfig[0], goalconfig[1], goalconfig[2]])
        handles.append(env.plot3(points=array((( goalconfig[0], goalconfig[1] , 0 ))),pointsize=0.05,colors=array(((0,1,0))),drawstyle=1))
        #### Draw your path in the openrave here (see /usr/lib/python2.7/dist-packages/openravepy/_openravepy_0_8/examples/tutorial_plotting.py for examples)
        path = solver.algorithm(startconfig, goal, n_connected, heuristic , env, robot)
        robot.SetActiveDOFValues(startconfig)
        # #### Draw the X and Y components of the configurations explored by A*
        for State in path:
            handles.append(env.plot3(points=array((( State.point.x, State.point.y , 0 ))),pointsize=0.03,colors=array(((0,0,0))),drawstyle=1))

        #### Now that you have computed a path, execute it on the robot using the controller. You will need to convert it into an openrave trajectory. You can set any reasonable timing for the configurations in the path. Then, execute the trajectory using robot.GetController().SetPath(mypath);
        traj = RaveCreateTrajectory(env, '')
        config = robot.GetActiveConfigurationSpecification('linear')
        config.AddDeltaTimeGroup()

        traj.Init(config)

        finalPath = list()

        for i, point in enumerate(path):
            State = path[i]
            finalPath.append([State.point.x, State.point.y, State.point.o, i*0.002])

        for i, p in enumerate(finalPath):
            traj.Insert(i, p, config, True)
        robot.GetController().SetPath(traj)
        planningutils.RetimeActiveDOFTrajectory(traj, robot, hastimestamps=False, maxvelmult=3, maxaccelmult=1)
        print 'Time for trajectory planning from start to goal: ', traj.GetDuration(),'s'

        #### END OF YOUR CODE ###
    waitrobot(robot)

    raw_input("Press enter to exit...")
