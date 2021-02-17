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
p.setRealTimeSimulation(1)

# define world
p.setGravity(0,0,-10) 
planeID = p.loadURDF("plane.urdf")

# define environment
ballStartPos = [0.1, -0.5, 0]
ballStartPos2 = [1.1, 0.5, 0]
ballStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
soccerBallId = p.loadURDF("soccerball.urdf", ballStartPos, ballStartOrientation, globalScaling=0.12)
soccerBallId2 = p.loadURDF("soccerball.urdf", ballStartPos2, ballStartOrientation, globalScaling=0.12)

p.changeDynamics(soccerBallId,-1,linearDamping=0, angularDamping=0, rollingFriction=5.001, spinningFriction=.001,mass=0.001, lateralFriction=200)
p.changeDynamics(soccerBallId2,-1,linearDamping=0, angularDamping=0, rollingFriction=5.001, spinningFriction=.001,mass=0.001, lateralFriction=200)

# setup ur5 with robotiq 140
robotStartPos = [0,0,0.0]
robot2StartPos = [1,1,0.0]
robotStartOrn = p.getQuaternionFromEuler([0,0,0])

# Robot at [0,0,0]
robotID = p.loadURDF(sisbotUrdfPath, robotStartPos, robotStartOrn,useFixedBase = True,
                     flags=p.URDF_USE_INERTIA_FROM_FILE)

# Robot at [1,1,0]
robot2ID = p.loadURDF(sisbotUrdfPath, robot2StartPos, robotStartOrn,useFixedBase = True,
                     flags=p.URDF_USE_INERTIA_FROM_FILE)

# Increase surface friction of robot to make gripping ball easier
p.changeDynamics(robotID,6,lateralFriction=20)
p.changeDynamics(robot2ID,6,lateralFriction=20)

joints, controlRobotiqC2, controlJoints, mimicParentName = utils_ur5_robotiq140.setup_sisbot(p, robotID)
joints2, controlRobotiq2C2, controlJoints2, mimicParentName2 = utils_ur5_robotiq140.setup_sisbot(p, robot2ID)
eefID = 7 # ee_link
eef2ID = 7 # ee_link

# Start simulation
ABSE = lambda a,b: abs(a-b)

# Set IK damping for robot arm and gripper
jd = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1,0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
jd = jd*10

# List of desired poses in order in the format:
# [X_Position, Y_Position, Z_Position, Gripper_Opening_Length]
pose_list = [[0.1,-0.5,0.23,0.085],[0.1,-0.5,0.23,0.067], [0.3,-0.2,0.3,0.067], [0.4,0.1,0.3,0.067],[0.4,0.1,0.25,0.085], [0.4,0.1,0.4,0.085]]
pose_list2 = [[1.1,0.5,0.23,0.085],[1.1,0.5,0.23,0.067], [1.3,0.8,0.3,0.067], [1.4,1.1,0.3,0.067],[1.4,1.1,0.25,0.085], [1.4,1.1,0.4,0.085]]

userParams = dict()
userParams2 = dict()


def move_to_pose(pose_list, current_pose_idx, rXYZ, change_pose_flag):
    if current_pose_idx  == len(pose_list) - 1:
        pass
    elif pose_list[current_pose_idx][0] - rXYZ[0] < 0.01 and pose_list[current_pose_idx][1] - rXYZ[1] < 0.01 and pose_list[current_pose_idx][2] - rXYZ[2] < 0.01:
        if change_pose_flag:
            current_pose_idx += 1
            time.sleep(0.5)
            change_pose_flag = 0
        change_pose_flag = 1
    return pose_list[current_pose_idx][0], pose_list[current_pose_idx][1], pose_list[current_pose_idx][2], pose_list[current_pose_idx][3], current_pose_idx, change_pose_flag

try:
    flag = True

    # Robot Arm 2
    x2 = 1.1
    y2 = 0.5
    z2 = 1
    roll2 =  0
    pitch2  = 1.57
    yaw2 = -1.57 
    gripper_opening_length_control2 = 0.085
    orn2 = p.getQuaternionFromEuler([roll2, pitch2, yaw2])
    gripper_opening_length2 = gripper_opening_length_control2

    # Joint space 
    userParams2[0] = -1.57 # shoulder_pan_joint
    userParams2[1] = -1.57 # shoulder_lift_joint
    userParams2[2] = 1.57 # elbow_joint
    userParams2[3] = -1.57 # wrist_1_joint
    userParams2[4] = -1.57  # wrist_2_joint
    userParams2[5] = -0 # wrist_3_joint

    control_cnt2 = 0;
    current_pose_idx2 = 0
    change_pose_flag2 = 0

    # Robot Arm 1
    x = 0.1
    y = -0.5
    z = 1
    roll = 0
    pitch = 1.57
    yaw = -1.57
    gripper_opening_length_control = 0.085
    orn = p.getQuaternionFromEuler([roll, pitch, yaw])

    userParams[0]  = -1.57 # shoulder_pan_joint
    userParams[1]  = -1.57 # shoulder_lift_joint
    userParams[2]  = 1.57 # elbow_joint
    userParams[3]  = -1.57 # wrist_1_joint
    userParams[4]  = -1.57 # wrist_2_joint
    userParams[5]  = -0    # wrist_3_joint

    control_cnt = 0;
    change_pose_flag = 0
    current_pose_idx = 0
    gripper_opening_length = gripper_opening_length_control
    while(flag):

        # KINEMATICS SOLVER ----------------------------------------------------------------------------------------------------------------------
        # Robot Arm 2
        # apply IK for robot arm 2
        gripper_opening_angle2 = 0.715 - math.asin((gripper_opening_length2 - 0.010) / 0.1143)    # angle calculation
        jointPose2 = p.calculateInverseKinematics(robot2ID, eef2ID, [x2,y2,z2],orn2,jointDamping=jd)
        for i2, name2 in enumerate(controlJoints2):
    
            joint2 = joints2[name2]
            pose2 = jointPose2[i2]
            # read joint value
            if i2 != 6:
                pose12 = userParams2[i2]

            if name2==mimicParentName:
                controlRobotiq2C2(controlMode=p.POSITION_CONTROL, targetPosition=gripper_opening_angle2)
            else:
                if control_cnt2 < 100:
                    # control robot joints
                    p.setJointMotorControl2(robot2ID, joint2.id, p.POSITION_CONTROL,
                                        targetPosition=pose12, force=joint2.maxForce, 
                                        maxVelocity=joint2.maxVelocity)
                else:
                    # control robot end-effector
                    p.setJointMotorControl2(robot2ID, joint2.id, p.POSITION_CONTROL,
                                        targetPosition=pose2, force=joint2.maxForce, 
                                        maxVelocity=joint2.maxVelocity)

        control_cnt2 = control_cnt2 + 1
        
        # apply IK for robot arm 1
        gripper_opening_angle = 0.715 - math.asin((gripper_opening_length - 0.010) / 0.1143)    # angle calculation
        jointPose = p.calculateInverseKinematics(robotID, eefID, [x,y,z],orn,jointDamping=jd)
        for i, name in enumerate(controlJoints):
    
            joint = joints[name]
            pose = jointPose[i]
            # read joint value
            if i != 6:
                pose1 = userParams[i]

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

        # END KINEMATICS SOLVER ----------------------------------------------------------------------------------------------------------------------

        # Get current robots poses
        rXYZ2 = p.getLinkState(robot2ID, eef2ID)[0] # real XYZ
        rxyzw2= p.getLinkState(robot2ID, eef2ID)[1] # real rpy
        rroll2, rpitch2, ryaw2 = p.getEulerFromQuaternion(rxyzw2)

        rXYZ = p.getLinkState(robotID, eefID)[0] # real XYZ
        rxyzw = p.getLinkState(robotID, eefID)[1] # real rpy
        rroll, rpitch, ryaw = p.getEulerFromQuaternion(rxyzw)

        x,y,z,gripper_opening_length, current_pose_idx, change_pose_flag = move_to_pose(pose_list, current_pose_idx, rXYZ, change_pose_flag)
        x2,y2,z2,gripper_opening_length2, current_pose_idx2, change_pose_flag2 = move_to_pose(pose_list2, current_pose_idx2, rXYZ2, change_pose_flag2)
        
    p.disconnect()
except KeyError:
    p.disconnect()
