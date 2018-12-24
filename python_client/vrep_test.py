import vrep
import sys
import ctypes
import numpy as np

# Read the data.txt file
data = np.genfromtxt("data.txt")
data = np.around(data, decimals=4)
data = data.tolist()
tryConfig = [-1.2172927856445312, -1.5707786083221436, 0.1, 2.790855407714844e-01, 0.269050598144531, 1.384185791015625]
print("Start Program")
vrep.simxFinish(-1)  # close all opened connections
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, -500000, 5)  # Connect to V-REP

if clientID != -1:
    print('Connected to remote API server')
    vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot_wait)


    # Option1. Add a Robot by path
    # vrep.simxLoadModel(clientID, r"C:\Program Files\V-REP3\V-REP_PRO_EDU\models\robots\non-mobile\UR5.ttm",1,
    #                   vrep.simx_opmode_oneshot_wait)
    # Option2. Move dummy
    # res, target_handle = vrep.simxGetObjectHandle(clientID, 'UR5_ikTarget', vrep.simx_opmode_blocking)
    # vrep.simxSetObjectPosition(clientID, target_handle, -1, [x, y, z], vrep.simx_opmode_oneshot)
    #
    # Option3. Move robot with given joint angle NO PATH PLANNING
    # jointHandles = [-1, -1, -1, -1, -1, -1]
    # targetPos1 = [90 * 3.14 / 180, 90 * 3.14 / 180, -90 * 3.14 / 180, 90 * 3.14 / 180, 90 * 3.14 / 180, 90 * 3.14 / 180]
    #
    # for j in range(0, len(data), 1):
    #     for i in range(0, 6, 1):
    #         res, jointHandles[i] = vrep.simxGetObjectHandle(clientID, 'UR5_joint' + str(i + 1), vrep.simx_opmode_blocking)
    #         res= vrep.simxSetJointTargetPosition(clientID, jointHandles[i], data[j][i], vrep.simx_opmode_blocking)


    emptyBuff = bytearray()
    # Step1. Get robot and target handle
    res, robotHandle=vrep.simxGetObjectHandle(clientID,'UR5', vrep.simx_opmode_oneshot_wait)

    # Step2. Retrieve the poses (i.e. transformation matrices, 12 values, last row is implicit) of some dummies in the scene

    # Step3. Get the robot initial state:
    res,retInts,robotInitialState,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID,'remoteApiCommandServer',
                                                                                   vrep.sim_scripttype_childscript,
                                                                                   'getRobotState', [robotHandle], [], [],
                                                                                   emptyBuff, vrep.simx_opmode_oneshot_wait)

    # Set Some parameters:
    approachVector = [0, 0, 1]  # often a linear approach is required. This should also be part of the calculations when selecting an appropriate state for a given pose
    maxConfigsForDesiredPose = 20  # we will try to find 10 different states corresponding to the goal pose and order them according to distance from initial state
    maxTrialsForConfigSearch = 300  # a parameter needed for finding appropriate goal states
    searchCount = 4  # how many times OMPL will run for a given task
    minConfigsForPathPlanningPath = 40 # interpolation states for the OMPL path
    minConfigsForIkPath = 10  # interpolation states for the linear approach path
    collisionChecking = 0  # whether collision checking is on or off

    # Step4. Do the path planning here (between a start state and a goal pose, including a linear approach phase):
    for target_index in range (0, len(data), 1):
        emptyBuff = bytearray()
        res, robotHandle = vrep.simxGetObjectHandle(clientID, 'UR5', vrep.simx_opmode_oneshot_wait)
        # Get the robot current state:
        res, retInts, robotCurrentConfig, retStrings, retBuffer = vrep.simxCallScriptFunction(clientID,
                                                                                              'remoteApiCommandServer',
                                                                                              vrep.sim_scripttype_childscript,
                                                                                              'getRobotState',
                                                                                              [robotHandle],
                                                                                              [], [], emptyBuff,
                                                                                              vrep.simx_opmode_oneshot_wait)

        inInts = [robotHandle, collisionChecking, minConfigsForPathPlanningPath, searchCount]
        inFloats = robotCurrentConfig + data[target_index] # Put desired target pose here

        res, retInts, path, retStrings, retBuffer = vrep.simxCallScriptFunction(clientID, 'remoteApiCommandServer',
                                                                                vrep.sim_scripttype_childscript,
                                                                                'findPath_goalIsState', inInts, inFloats, [],
                                                                                emptyBuff, vrep.simx_opmode_oneshot_wait)
        print(path)
        # Step5.
        if (res == 0) and len(path) > 0:
            # Visualize the path:
            res, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(clientID, 'remoteApiCommandServer',
                                                                                             vrep.sim_scripttype_childscript,
                                                                                             'visualizePath',
                                                                                             [robotHandle, 255, 0, 255], path,
                                                                                             [], emptyBuff,
                                                                                             vrep.simx_opmode_oneshot_wait)
            lineHandle = retInts[0]

            # Make the robot follow the path:
            res, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(clientID, 'remoteApiCommandServer',
                                                                                             vrep.sim_scripttype_childscript,
                                                                                             'runThroughPath', [robotHandle],
                                                                                             path, [], emptyBuff,
                                                                                             vrep.simx_opmode_oneshot_wait)

            # Wait until the end of the movement:
            runningPath = True
            while runningPath:
                res, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(clientID,
                                                                                                 'remoteApiCommandServer',
                                                                                                 vrep.sim_scripttype_childscript,
                                                                                                 'isRunningThroughPath',
                                                                                                 [robotHandle], [], [],
                                                                                                 emptyBuff,
                                                                                                 vrep.simx_opmode_oneshot_wait)
                runningPath = retInts[0] == 1

            # Clear the path visualization:
            res, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(clientID, 'remoteApiCommandServer',
                                                                                             vrep.sim_scripttype_childscript,
                                                                                             'removeLine', [lineHandle], [], [],
                                                                                             emptyBuff,
                                                                                             vrep.simx_opmode_oneshot_wait)

    vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait)
    print('Program ended')


else:
    print('Failed connecting to remote API server')