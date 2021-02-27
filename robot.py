import time
import pybullet as p
import pybullet_data
import utils_ur_robotiq
import math
# import pyquaternion

# def calculate_angular_difference(quat_1, quat_2):


class Robot:
    def __init__(
        self,
        physics_client,
        robot_id,
        x_init,
        y_init,
        z_init,
        roll_init,
        pitch_init,
        yaw_init,
        gripper_opening_length_init,
        pose_list=[],
        pose_transition_time=0.9,
        eef_id=7,
        robot_pose_tolerance=0.01,
    ):
        # PyBullet
        self.p = physics_client
        self.robot_id = robot_id
        self.eef_id = eef_id

        # Robot position
        self.target_x = x_init
        self.target_y = y_init
        self.target_z = z_init

        # Robot orientation
        self.target_roll = roll_init
        self.target_pitch = pitch_init
        self.target_yaw = yaw_init
        self.target_orn = self.p.getQuaternionFromEuler(
            [self.target_roll, self.target_pitch, self.target_yaw]
        )
        
        # Robot gripper
        self.gripper_opening_length = gripper_opening_length_init

        # Kinematics helpers
        (
            self.joints,
            self.controlRobotiqC2,
            self.controlJoints,
            self.gripperParentName,
        ) = utils_ur_robotiq.setup_sisbot(self.p, self.robot_id)
        self.joint_damping = [0.1] * 12

        # Robot pose
        self.pose_list = pose_list
        self.robot_pose_tolerance = robot_pose_tolerance
        self.current_pose = 0
        self.last_pose_completion_time = 0
        self.pose_transition_time = pose_transition_time

    def movel(self):
        # Calculate IK
        self.calculate_robot_pose()
        self.select_target_pose()
        jointPose = self.p.calculateInverseKinematics(
            self.robot_id,
            self.eef_id,
            [self.target_x, self.target_y, self.target_z],
            self.target_orn,
            jointDamping=self.joint_damping,
        )

        # Iterate through robot joints and apply calculated IK from ^^
        # TO-DO: seperate IK calculation and motoring
        for i, name in enumerate(self.controlJoints):
            joint = self.joints[name]
            pose = jointPose[i]

            # Motors gripper & arm joints seperately
            if name == self.gripperParentName:
                self.controlRobotiqC2(
                    controlMode=self.p.POSITION_CONTROL,
                    targetPosition=self.gripper_opening_angle,
                )
            else:
                p.setJointMotorControl2(
                    self.robot_id,
                    joint.id,
                    self.p.POSITION_CONTROL,
                    targetPosition=pose,
                    force=joint.maxForce,
                    maxVelocity=joint.maxVelocity,
                )
        return

    def calculate_robot_pose(self):
        self.gripper_opening_angle = 0.715 - math.asin(
            (self.gripper_opening_length - 0.010) / 0.1143
        )
        # self.target_orn = self.p.getQuaternionFromEuler(
        #     [self.target_roll, self.target_pitch, self.target_yaw]
        # )

        # Position pose
        self.rXYZ = self.p.getLinkState(self.robot_id, self.eef_id)[0]

        # Rotation pose
        self.rxyzw = self.p.getLinkState(self.robot_id, self.eef_id)[1]
        # print(self.rxyzw)
        return
    def print_pose(self):
        print(f'rXYZ: {self.rXYZ}, Gripper: {self.gripper_opening_length}')
        return

    def select_target_pose(self):
        # If last pose in pose_list is achieved then just remain there
        print(f'{self.target_orn=} {self.rxyzw=}')
        if self.current_pose == len(self.pose_list) - 1:
            return

        # Check if target pose is achieved i.e.: target_pose - target_pose is within tolerance?
        # Currently only working with gripper & position
        # TO-DO: implement orientation pose
        if (
            # Position
            self.pose_list[self.current_pose][0] - self.rXYZ[0] < 0.01
            and self.pose_list[self.current_pose][1] - self.rXYZ[1] < 0.01
            and self.pose_list[self.current_pose][2] - self.rXYZ[2] < 0.01
            # Orientation
            and self.rxyzw[0] - self.target_orn[0] < 0.05
            and self.rxyzw[1] - self.target_orn[1] < 0.05
            and self.rxyzw[2] - self.target_orn[2] < 0.05
            and self.rxyzw[3] - self.target_orn[3] < 0.05
        ):

            # Logic to hold pose for X seconds before moving to next pose
            if self.last_pose_completion_time == 0:
                self.last_pose_completion_time = time.time()

            # If target_pose is achieved and the pose has been held for X seconds:
            # Increment the current pose index and update the target XYZ and gripper length with the next pose in list
            if time.time() - self.last_pose_completion_time > self.pose_transition_time:
                self.current_pose += 1

                self.target_x = self.pose_list[self.current_pose][0]
                self.target_y = self.pose_list[self.current_pose][1]
                self.target_z = self.pose_list[self.current_pose][2]
                self.target_orn = p.getQuaternionFromEuler(self.pose_list[self.current_pose][3])
                self.gripper_opening_length = self.pose_list[self.current_pose][4]

                self.last_pose_completion_time = 0
        return
