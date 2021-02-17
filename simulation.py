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
robotStartPos = [0, 0, 0]
robotStartOrn = p.getQuaternionFromEuler([0, 0, 0])
robotID = p.loadURDF(
    ur3_urdf_path,
    robotStartPos,
    robotStartOrn,
    useFixedBase=True,
    flags=p.URDF_USE_INERTIA_FROM_FILE,
)
eefID = 7  # end effector link

# Increase surface friction of robot to make gripping ball easier
p.changeDynamics(robotID, 6, lateralFriction=20)

# Start simulation
ABSE = lambda a, b: abs(a - b)

try:
    flag = True
    pose_list1 = [
        [0.1, -0.5, 0.23, 0.085],
        [0.1, -0.5, 0.23, 0.067],
        [0.3, -0.2, 0.3, 0.067],
        [0.4, 0.1, 0.3, 0.067],
        [0.4, 0.1, 0.25, 0.085],
        [0.4, 0.1, 0.4, 0.085],
        [0.4, 0.1, 0.4, 0.025],
    ]

    ur3 = Robot(
        p, robotID, 0.1, -0.5, 0.23, 0, 1.57, -1.57, 0.085, pose_list=pose_list1
    )
    while 1:
        # apply IK for robot arm 1
        ur3.calculate_robot_pose()
        ur3.movel()
        ur3.select_target_pose()
    p.disconnect()
except KeyError:
    p.disconnect()
