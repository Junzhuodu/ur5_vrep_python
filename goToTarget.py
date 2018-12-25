import vrep
import numpy as np
import math

# data = np.genfromtxt("data.txt")
# data = np.around(data, decimals=4)
# data = data.tolist()

print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19997,True,True,-500000,5) # Connect to V-REP, set a very large time-out for blocking commands
if clientID!=-1:
    print ('Connected to remote API server')

    emptyBuff = bytearray()


    #Start the simulation:
    vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot_wait)

    # Load a robot instance: res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'loadRobot',[],[0,0,0,0],['d:/v_rep/qrelease/release/test.ttm'],emptyBuff,vrep.simx_opmode_oneshot_wait)
    #    robotHandle=retInts[0]
    
    # Retrieve some handles:
    res,robotHandle=vrep.simxGetObjectHandle(clientID,'UR5#',vrep.simx_opmode_oneshot_wait)  
    res,target1=vrep.simxGetObjectHandle(clientID,'testPose1#',vrep.simx_opmode_oneshot_wait)

    #### original four points
    tryConfig_1 = [-1.0110,  -1.1291,    0.7916,    0.3375,    2.1395,    0.7224]
    tryConfig_2 = [0.7278,   -1.8713,   -1.2708,   0.0005,    0.8948,   -2.4192]
    tryConfig_3 = [-0.5999,   -2.3992,   -1.7141,    0.9717,    2.2225,   -2.4192]
    tryConfig_4 = [-1.0094,   -1.1576,    1.3857,   -0.2281,    2.1271,   -2.4192]

    ###right side (1,1)
    tryConfig1_1_r= [0.9226,   -2.3992,   -1.7141,    0.9717,    0.700,   -2.4192] 

    ##left side (2,1)
    tryConfig2_1_l = [-2.5410,   -1.1576,    1.3857,   -0.2281,    0.6095,   -2.4192]
    ###left side(2,2)
    tryConfig2_2_l = [-2.2910,   -1.4576,    1.6857,   -0.2281,    0.8595,   -2.4192]

    ###left side (1,1)
    tryConfig1_1_l = [-2.5410,   -0.9576,    1.5857,   -0.6281,    0.6095,   -2.4192]


    #### deep shelf
    tryConfig_d_1 = [2.7915,-0.8713,0.0708,0.8005,1.2208,-2.4192]
    tryConfig_origin = [0.0059814453125, -1.5647833347320557, 0.0019979476928710938, 0.006038188934326172, 0.005944967269897461, -0.0059778690338134766]
    tryConfig_d_2 = [-0.1,-1.8013,-1.6908,0.4521,1.67,-2.4192]

    # Retrieve the poses (i.e. transformation matrices, 12 values, last row is implicit) of some dummies in the scene
    res,retInts,target1Pose,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'getObjectPose',[target1],[],[],emptyBuff,vrep.simx_opmode_oneshot_wait)
    


    res, retInts, robotInitialState, retStrings, retBuffer = vrep.simxCallScriptFunction(clientID, 'remoteApiCommandServer', vrep.sim_scripttype_childscript,'getRobotState',[robotHandle],[],[],emptyBuff,vrep.simx_opmode_oneshot_wait)
    
     # Some parameters:
    approachVector = [0, 0, 0]  #often a linear approach is required. This should also be part of the calculations when selecting an appropriate state for a given pose
    maxConfigsForDesiredPose = 100  # we will try to find 10 different states corresponding to the goal pose and order them according to distance from initial state
    maxTrialsForConfigSearch=300 # a parameter needed for finding appropriate goal states
    searchCount=8 # how many times OMPL will run for a given task
    minConfigsForPathPlanningPath=400 # interpolation states for the OMPL path
    minConfigsForIkPath=100 # interpolation states for the linear approach path
    collisionChecking=1 # whether collision checking is on or off

    inInts=[robotHandle,collisionChecking,minConfigsForPathPlanningPath,searchCount]
    
    res, retInts, robotCurrentConfig, retStrings ,retBuffer = vrep.simxCallScriptFunction(clientID, 'remoteApiCommandServer', vrep.sim_scripttype_childscript, 'getRobotState',[robotHandle],[],[],emptyBuff,vrep.simx_opmode_oneshot_wait)
    
    
    inFloats = robotCurrentConfig + tryConfig_1
    # inFloats = robotCurrentConfig + tryConfig_d_1

    res,retInts,path,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'findPath_goalIsState',inInts,inFloats,[],emptyBuff,vrep.simx_opmode_oneshot_wait)
    if (res==0) and len(path)>0:
        # Visualize the path:
        res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'visualizePath',[robotHandle,255,0,255],path,[],emptyBuff,vrep.simx_opmode_oneshot_wait)
        lineHandle=retInts[0]
        

        # Make the robot follow the path:
        res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'runThroughPath',[robotHandle],path,[],emptyBuff,vrep.simx_opmode_oneshot_wait)
        
        # Wait until the end of the movement:
        runningPath=True
        while runningPath:
            res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'isRunningThroughPath',[robotHandle],[],[],emptyBuff,vrep.simx_opmode_oneshot_wait)
            runningPath=retInts[0]==1
            
        # Clear the path visualization:
        res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',vrep.sim_scripttype_childscript,'removeLine',[lineHandle],[],[],emptyBuff,vrep.simx_opmode_oneshot_wait)


       


    # Stop simulation:
    vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait)

    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')
