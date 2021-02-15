import os
import time
import pdb
import pybullet as p
import pybullet_data
import utils_ur5_robotiq140
from collections import deque
import numpy as np
import math
import matplotlib.pyplot as plt

serverMode = p.GUI # GUI/DIRECT
sisbotUrdfPath = "./urdf/ur3_robotiq_140.urdf"
# connect to engine servers
physicsClient = p.connect(serverMode)
# add search path for loadURDFs
p.setAdditionalSearchPath(pybullet_data.getDataPath())
#p.getCameraImage(640,480)
p.setRealTimeSimulation(1)

# define world
p.setGravity(0,0,-10) 
planeID = p.loadURDF("plane.urdf")

# define environment
deskStartPos = [0.1, -0.5, 0]
deskStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
boxId = p.loadURDF("soccerball.urdf", deskStartPos, deskStartOrientation, globalScaling=0.12)
p.changeDynamics(boxId,-1,linearDamping=0, angularDamping=0, rollingFriction=5.001, spinningFriction=.001,mass=0.001, lateralFriction=200)

# setup ur5 with robotiq 140
robotStartPos = [0,0,0.0]
robot2StartPos = [1,1,0.0]
robotStartOrn = p.getQuaternionFromEuler([0,0,0])
print("----------------------------------------")
print("Loading robot from {}".format(sisbotUrdfPath))
robotID = p.loadURDF(sisbotUrdfPath, robotStartPos, robotStartOrn,useFixedBase = True,
                     flags=p.URDF_USE_INERTIA_FROM_FILE)
robot2ID = p.loadURDF(sisbotUrdfPath, robot2StartPos, robotStartOrn,useFixedBase = True,
                     flags=p.URDF_USE_INERTIA_FROM_FILE)
p.changeDynamics(robotID,6,lateralFriction=20)
p.changeDynamics(robot2ID,6,lateralFriction=20)
joints, controlRobotiqC2, controlJoints, mimicParentName = utils_ur5_robotiq140.setup_sisbot(p, robotID)
joints2, controlRobotiq2C2, controlJoints2, mimicParentName2 = utils_ur5_robotiq140.setup_sisbot(p, robot2ID)
eefID = 7 # ee_link
eef2ID = 7 # ee_link

# start simulation
ABSE = lambda a,b: abs(a-b)

# set damping for robot arm and gripper
jd = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1,0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
jd = jd*10

userParams = dict()
userParams2 = dict()

def move_to_pose(pose_list, current_pose_idx, rXYZ, wait_counter):
    print(f'{current_pose_idx}, {len(pose_list)}')
    if current_pose_idx  == len(pose_list) - 1:
        pass
    elif pose_list[current_pose_idx][0] - rXYZ[0] < 0.01 and pose_list[current_pose_idx][1] - rXYZ[1] < 0.01 and pose_list[current_pose_idx][2] - rXYZ[2] < 0.01:
        if wait_counter > 80:
            current_pose_idx += 1
            wait_counter = 0
        wait_counter += 1
    return pose_list[current_pose_idx][0], pose_list[current_pose_idx][1], pose_list[current_pose_idx][2], pose_list[current_pose_idx][3], current_pose_idx, wait_counter

try:
    flag = True
    # custom sliders to tune parameters (name of the parameter,range,initial value)
    # Task space (Cartesian space)
    # Robot Arm 2
    xin2 = p.addUserDebugParameter("x2", 0,2, 0.8)
    yin2 = p.addUserDebugParameter("y2", 0,2, 1.156)
    zin2 = p.addUserDebugParameter("z2", 0.0, 1, 0.576)
    rollId2 = p.addUserDebugParameter("roll2", -3.14, 3.14, 0) #-1.57 yaw
    pitchId2 = p.addUserDebugParameter("pitch2", -3.14, 3.14, 1.57)
    yawId2 = p.addUserDebugParameter("yaw2", -3.14, 3.14, -1.57) # -3.14 pitch
    gripper_opening_length_control2 = p.addUserDebugParameter("gripper_opening_length2",0,0.085,0.085)

    # Joint space 
    userParams2[0] = p.addUserDebugParameter("shoulder_pan_joint", -3.14, 3.14, -1.57)
    userParams2[1] = p.addUserDebugParameter("shoulder_lift_joint", -3.14, 3.14, -1.57)
    userParams2[2] = p.addUserDebugParameter("elbow_joint", -3.14, 3.14, 1.57)
    userParams2[3] = p.addUserDebugParameter("wrist_1_joint",-3.14, 3.14, -1.57)
    userParams2[4] = p.addUserDebugParameter("wrist_2_joint", -3.14, 3.14, -1.57)
    userParams2[5] = p.addUserDebugParameter("wrist_3_joint", -3.14, 3.14, 0)   


    # Robot Arm 1
    xin = p.addUserDebugParameter("x", -3.14, 3.14, 0.1)
    yin = p.addUserDebugParameter("y", -3.14, 3.14, -0.5)
    zin = p.addUserDebugParameter("z", 0.0, 0.5, 1)
    rollId = p.addUserDebugParameter("roll", -3.14, 3.14, 0) #-1.57 yaw
    pitchId = p.addUserDebugParameter("pitch", -3.14, 3.14, 1.57)
    yawId = p.addUserDebugParameter("yaw", -3.14, 3.14, -1.57) # -3.14 pitch
    gripper_opening_length_control = p.addUserDebugParameter("gripper_opening_length",0,0.085,0.085)

    # Joint space 
    userParams[0] = p.addUserDebugParameter("shoulder_pan_joint", -3.14, 3.14, 0)
    userParams[1] = p.addUserDebugParameter("shoulder_lift_joint", -3.14, 3.14, 0)
    userParams[2] = p.addUserDebugParameter("elbow_joint", -3.14, 3.14, 0)
    userParams[3] = p.addUserDebugParameter("wrist_1_joint",-3.14, 3.14, -1.57)
    userParams[4] = p.addUserDebugParameter("wrist_2_joint", -3.14, 3.14, -1.57)
    userParams[5] = p.addUserDebugParameter("wrist_3_joint", -3.14, 3.14, 0)   

    # Robot Arm 1
    x = p.readUserDebugParameter(xin)
    y = p.readUserDebugParameter(yin)
    z = p.readUserDebugParameter(zin)
    roll = p.readUserDebugParameter(rollId)
    pitch = p.readUserDebugParameter(pitchId)
    yaw = p.readUserDebugParameter(yawId)
    orn = p.getQuaternionFromEuler([roll, pitch, yaw])
    control_cnt = 0;
    current_pose_idx = 0
    wait_counter = 0

    # time.sleep(25)
    gripper_opening_length = p.readUserDebugParameter(gripper_opening_length_control)
    gripper_opening_length2 = p.readUserDebugParameter(gripper_opening_length_control2)
    while(flag):

        # Robot Arm 2
        x2 = p.readUserDebugParameter(xin2)
        y2 = p.readUserDebugParameter(yin2)
        z2 = p.readUserDebugParameter(zin2)
        roll2 = p.readUserDebugParameter(rollId2)
        pitch2 = p.readUserDebugParameter(pitchId2)
        yaw2 = p.readUserDebugParameter(yawId2)
        orn2 = p.getQuaternionFromEuler([roll2, pitch2, yaw2])
        control_cnt2 = 0;

        # apply IK for robot arm 2
        gripper_opening_angle2 = 0.715 - math.asin((gripper_opening_length2 - 0.010) / 0.1143)    # angle calculation
        jointPose2 = p.calculateInverseKinematics(robot2ID, eef2ID, [x2,y2,z2],orn2,jointDamping=jd)
        for i2, name2 in enumerate(controlJoints2):
    
            joint2 = joints2[name2]
            pose2 = jointPose2[i2]
            # read joint value
            if i2 != 6:
                pose12 = p.readUserDebugParameter(userParams2[i2])

            if name2==mimicParentName:
                controlRobotiq2C2(controlMode=p.POSITION_CONTROL, targetPosition=gripper_opening_angle2)
            else:
                if control_cnt2 < 100:
                    # control robot joints
                    p.setJointMotorControl2(robot2ID, joint2.id, p.POSITION_CONTROL,
                                        targetPosition=pose2, force=joint2.maxForce, 
                                        maxVelocity=joint2.maxVelocity)
                else:
                    # control robot end-effector
                    p.setJointMotorControl2(robot2ID, joint2.id, p.POSITION_CONTROL,
                                        targetPosition=pose2, force=joint2.maxForce, 
                                        maxVelocity=joint2.maxVelocity)

        control_cnt2 = control_cnt2 + 1
        rXYZ2 = p.getLinkState(robot2ID, eef2ID)[0] # real XYZ
        rxyzw2= p.getLinkState(robot2ID, eef2ID)[1] # real rpy
        rroll2, rpitch2, ryaw2 = p.getEulerFromQuaternion(rxyzw2)
        
        # apply IK for robot arm 1
        gripper_opening_angle = 0.715 - math.asin((gripper_opening_length - 0.010) / 0.1143)    # angle calculation
        jointPose = p.calculateInverseKinematics(robotID, eefID, [x,y,z],orn,jointDamping=jd)
        for i, name in enumerate(controlJoints):
    
            joint = joints[name]
            pose = jointPose[i]
            # read joint value
            if i != 6:
                pose1 = p.readUserDebugParameter(userParams[i])

            if name==mimicParentName:
                controlRobotiqC2(controlMode=p.POSITION_CONTROL, targetPosition=gripper_opening_angle)
            else:
                if control_cnt < 100:
                    # control robot joints
                    p.setJointMotorControl2(robotID, joint.id, p.POSITION_CONTROL,
                                        targetPosition=pose1, force=joint.maxForce, 
                                        maxVelocity=joint.maxVelocity)
                else:
                    # control robot end-effector
                    p.setJointMotorControl2(robotID, joint.id, p.POSITION_CONTROL,
                                        targetPosition=pose, force=joint.maxForce, 
                                        maxVelocity=joint.maxVelocity)
        control_cnt = control_cnt + 1
        rXYZ = p.getLinkState(robotID, eefID)[0] # real XYZ
        rxyzw = p.getLinkState(robotID, eefID)[1] # real rpy
        rroll, rpitch, ryaw = p.getEulerFromQuaternion(rxyzw)

        cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
        # print(cubePos,cubeOrn)
        # step 1: wait for init pose

        # Go to pose
        pose_list = [[0.1,-0.5,0.23,0.085],[0.1,-0.5,0.23,0.067], [0.3,-0.2,0.3,0.067], [0.4,0.1,0.3,0.067],[0.4,0.1,0.25,0.085], [0.4,0.1,0.4,0.085]]
        x,y,z,gripper_opening_length, current_pose_idx, wait_counter = move_to_pose(pose_list, current_pose_idx, rXYZ, wait_counter)
        print(current_pose_idx)


        p.stepSimulation()
        time.sleep(1/60)
        
    p.disconnect()
except KeyError:
    p.disconnect()
