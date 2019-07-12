#!/usr/bin/env python
# -*- coding: utf-8 -*-
#HW3 for EECS 598 Motion Planning
import time
import openravepy

#### YOUR IMPORTS GO HERE ####

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

def construct_path(path):
    path = path.split(",")
    del path[-1]
    Path = []
    count = 0
    temp = []
    for theta in path:
        if count < 7:
            # convert to float type
            temp.append(float(theta))
            count = count + 1
        if count == 7:
            Path.append(temp)
            count = 0
            temp = []
    return Path
def save_smooth_hist(hist):
    hist = hist.split(",")
    del hist[-1]
    Hist = []
    for item in hist:
        Hist.append(float(item));
        
    Hist = array(Hist)
    save('smooth_dtata', Hist)

def path_cost(path):
    cost = 0
    for i in range(len(path)-1):
        d = 0
        for j in range(7):
            d = d + (path[i+1][j] - path[i][j]) ** 2
        cost = cost + sqrt(d)
    return cost

def ConvertPathToTrajectory(robot,path=[]):
#Path should be of the form path = [q_1, q_2, q_3,...], where q_i = [x_i, y_i, theta_i]

    if not path:
        return None
    # Initialize trajectory
    traj = RaveCreateTrajectory(env,'') 
    traj.Init(robot.GetActiveConfigurationSpecification())
    for i in range(0,len(path)):
        traj.Insert(i,numpy.array(path[i]))
    # Move Robot Through Trajectory
    planningutils.RetimeActiveDOFTrajectory(traj,robot,hastimestamps=False,maxvelmult=0.5,maxaccelmult=0.5)
    #planningutils.RetimeAffineTrajectory(traj,maxvelocities=ones(3),maxaccelerations=5*ones(3))
    return traj

if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)

    env.Reset()        
    # load a scene from ProjectRoom environment XML file
    env.Load('hw3.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    ### INITIALIZE YOUR PLUGIN HERE ###
    RaveInitialize()
    RaveLoadPlugin('build/planner')
    planner = RaveCreateModule(env, 'planner')
    ### END INITIALIZING YOUR PLUGIN ###
   

    # tuck in the PR2's arms for driving
    tuckarms(env,robot);
  
    #set start config
    jointnames =['l_shoulder_pan_joint','l_shoulder_lift_joint','l_elbow_flex_joint','l_upper_arm_roll_joint','l_forearm_roll_joint','l_wrist_flex_joint','l_wrist_roll_joint']
    robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])      
    startconfig = [-0.15,0.075,-1.008,0,0,-0.11,0]
    robot.SetActiveDOFValues(startconfig);
    robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

    with env:
        goalconfig = [0.449,-0.201,-0.151,0,0,-0.11,0]
        ### YOUR CODE HERE ###
        ###call your plugin to plan, draw, and execute a path from the current configuration of the left arm to the goalconfig
        step_size = 0.05
        goal_bias = 0.2
        # set the initial and goal config of the robot
        planner.SendCommand('setInitialConfig start %f %f %f %f %f %f %f' % tuple(startconfig))
        planner.SendCommand('setGoalConfig goal %f %f %f %f %f %f %f' % tuple(goalconfig))
        planner.SendCommand('planner_config params %f %f' % tuple([goal_bias, step_size]))
        print "Planner: RRT connect"
        print "Step size: ", step_size
        print "Goal bias: ", 0.16
        # record the time here
        print "Planning path..."
        start = time.clock()
        path  = planner.SendCommand('plan_path')
        end = time.clock()
        print "Time to find the path is : ", end - start, 's'
        path = construct_path(path)
        N_nodes = float(planner.SendCommand('count_nodes'))
        cost_u = path_cost(path)
        print "The total number of nodes sampled is ", N_nodes
        print "The cost of the un-smoothed path is ", cost_u

        handles = []
        # plot the path
        for item in path:
            robot.SetActiveDOFValues(item)
            handles.append(env.plot3(points=robot.GetLinks()[49].GetTransform()[0:3,3],pointsize=0.01,colors=[1,0,0],drawstyle=1))
     

        # smooth the path
        print"Smoothing path..."
        start = time.clock()
        s_path  = planner.SendCommand('smooth_path')
        end = time.clock()
        smooth_time = end - start
        print "Time to smooth the path is : ", smooth_time, 's'
        #smooth_hist = planner.SendCommand('smooth_hist_print')
        #save_smooth_hist(smooth_hist)
        s_path = construct_path(s_path)
        # plot the path
        for item in s_path:
            robot.SetActiveDOFValues(item)
            handles.append(env.plot3(points=robot.GetLinks()[49].GetTransform()[0:3,3],pointsize=0.01,colors=[0,0,1],drawstyle=1))
        cost_s = path_cost(s_path)
        print "The cost of the smoothed path is ", cost_s
        # move the robot
        traj = ConvertPathToTrajectory(robot, s_path)

    # Execute the trajectory on the robot.
    if traj != None:
        robot.GetController().SetPath(traj)

        ### END OF YOUR CODE ###
    waitrobot(robot)

    raw_input("Press enter to exit...")

