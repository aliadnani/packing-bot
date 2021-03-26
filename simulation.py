import time
import pybullet as p
import pybullet_data
import utils_ur_robotiq
import math
from robot import Robot

serverMode = p.GUI  # GUI/DIRECT
ur3_urdf_path = "./urdf/ur3_robotiq_140.urdf"

# Initialize PyBullet
physicsClient = p.connect(serverMode)
# add search path for loadURDFs
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setRealTimeSimulation(1)

# Set up world
p.setGravity(0, 0, -10)
ground_id = p.loadURDF("plane.urdf")

# Set up environment

standStartPos = [0,0,0]

standStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
# standId = p.loadURDF(
#     "./base_link.stl", standStartPos, standStartOrientation
# )
# Import ball
ballStartPos = [0.1, -0.5, 0]
ballStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
soccerBallId = p.loadURDF(
    "soccerball.urdf", ballStartPos, ballStartOrientation, globalScaling=0.12
)
p.changeDynamics(
    soccerBallId,
    -1,
    linearDamping=0,
    angularDamping=0,
    rollingFriction=5.001,
    spinningFriction=0.001,
    mass=0.001,
    lateralFriction=200,
)

# Import Robot 1
robotStartPos = [-0.2, 0.14, 0.18]
robotStartOrn = p.getQuaternionFromEuler([-1.5708 ,3.1416 ,1.5708])
robotID = p.loadURDF(
    ur3_urdf_path,
    robotStartPos,
    robotStartOrn,
    useFixedBase=True,
    flags=p.URDF_USE_INERTIA_FROM_FILE|p.URDF_USE_SELF_COLLISION,
)
eefID = 7  # end effector link

# Import Robot 2
robotStartPos2 = [0.2, 0.14, 0.18]
robotStartOrn2 = p.getQuaternionFromEuler([-1.5708 ,3.1416 ,-1.5708])
robotID2 = p.loadURDF(
    ur3_urdf_path,
    robotStartPos2,
    robotStartOrn2,
    useFixedBase=True,
    flags=p.URDF_USE_INERTIA_FROM_FILE|p.URDF_USE_SELF_COLLISION,
)
eefID2 = 7  # end effector link

# Increase surface friction of robot to make gripping ball easier
p.changeDynamics(robotID, 6, lateralFriction=20)
p.changeDynamics(robotID2, 6, lateralFriction=20)

# Start simulation
ABSE = lambda a, b: abs(a - b)
flag  = True
[0,0,0]
try:
    pose_list = [
        [-0.1, -0.3, 0.23,[0,0,0], 0.085],
        [-0.1, -0.3, 0.23,[0,1,0], 0.085],
        [-0.1, -0.3, 0.23,[1,0,0], 0.085]
        # [0.1, -0.5, 0.23, 0.067],
        # [0.3, -0.2, 0.3, 0.067],
        # [0.4, 0.1, 0.3, 0.067],
        # [0.4, 0.1, 0.25, 0.085],
        # [0.4, 0.1, 0.4, 0.085],
        # [0.4, 0.1, 0.4, 0.025],
    ]

    pose_list2 = [
        [0.1, -0.3, 0.3,[0,1.57,1.57], 0.085],
        [0.1, -0.2, 0.3,[0,1.57,1.57], 0.085],
        [0.1, -0.3, 0.3,[0,1.57,1.57], 0.085]
    ]

    ur3 = Robot(
        physics_client=p,
        robot_id=robotID,
        x_init=0.1,
        y_init=-0.5,
        z_init=0.4,
        roll_init=0,
        pitch_init=1.57,
        yaw_init=-1.57,
        gripper_opening_length_init=0.085,
        pose_list=pose_list,
        pose_transition_time=1.5
    )

    ur3_2 = Robot(
        physics_client=p,
        robot_id=robotID2,
        x_init=0.5,
        y_init=-0.3,
        z_init=0.43,
        roll_init=0,
        pitch_init=1.57,
        yaw_init=-1.57,
        gripper_opening_length_init=0.085,
        pose_list=pose_list2,
        pose_transition_time=1.5
    )
    while 1:
        # ur3.calculate_robot_pose()
        # ur3_2.calculate_robot_pose()

        # ur3_2.select_target_pose()
        # ur3.select_target_pose()

        # ur3.movel()
        ur3_2.movel()


    p.disconnect()
except KeyError:
    p.disconnect()
