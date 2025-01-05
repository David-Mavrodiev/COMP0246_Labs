import rclpy
from rclpy.node import Node
import numpy as np
from numpy.typing import NDArray

from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion
from trajectory_msgs.msg import JointTrajectory

# Assuming transform_helpers.utils is available and implemented
from transform_helpers.utils import rotmat2q

class YoubotKinematicBase(Node):
    def __init__(self, tf_suffix=''):
        super().__init__('youbot_kinematic_base')
        self.tf_suffix = tf_suffix

        youbot_dh_parameters = {
            'a': [-0.033, 0.155, 0.135, +0.002, 0.0],
            'alpha': [np.pi / 2, 0.0, 0.0, np.pi / 2, np.pi],
            'd': [0.145, 0.0, 0.0, 0.0, -0.185],
            'theta': [np.pi, np.pi / 2, 0.0, -np.pi / 2, np.pi],
        }

        # youbot_dh_parameters = {
        #     'a': [
        #         -0.024,    # a1: Distance from base_link to arm_link_0 along X0
        #         0.033,     # a2: Distance from arm_link_1 to arm_link_2 along X1
        #         0.000,     # a3: Distance from arm_link_2 to arm_link_3 along X2
        #         0.000,     # a4: Distance from arm_link_3 to arm_link_4 along X3
        #         -0.002     # a5: Distance from arm_link_4 to arm_link_5 along X4
        #     ],
        #     'alpha': [
        #         np.pi / 2, # alpha1: 90 degrees
        #         0.0,       # alpha2: 0 degrees
        #         0.0,       # alpha3: 0 degrees
        #         np.pi / 2, # alpha4: 90 degrees
        #         np.pi      # alpha5: 180 degrees
        #     ],
        #     'd': [
        #         0.145,     # d1: Offset along Z0
        #         0.0,       # d2: Offset along Z1
        #         0.0,       # d3: Offset along Z2
        #         0.0,       # d4: Offset along Z3
        #         -0.185     # d5: Offset along Z4
        #     ],
        #     'theta': [
        #         2.96705972839,  # theta1: Joint 1 offset (~170 degrees)
        #         -1.1344640138,  # theta2: Joint 2 offset (~-65 degrees)
        #         2.54818070791,  # theta3: Joint 3 offset (~146 degrees)
        #         -1.78896248329, # theta4: Joint 4 offset (~-102.5 degrees)
        #         2.92342649709   # theta5: Joint 5 offset (~167.5 degrees)
        #     ]
        # }
        self.dh_params = youbot_dh_parameters.copy()

        self.current_joint_position = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

        self.joint_limit_min = np.array([-169 * np.pi / 180, -65 * np.pi / 180, -150 * np.pi / 180,
                                         -102.5 * np.pi / 180, -167.5 * np.pi / 180])
        self.joint_limit_max = np.array([169 * np.pi / 180, 90 * np.pi / 180, 146 * np.pi / 180,
                                         102.5 * np.pi / 180, 167.5 * np.pi / 180])

        # self.joint_limit_min = np.array([
        #     0.0,               # Joint 1: lower limit in radians
        #     0.0,               # Joint 2: lower limit in radians
        #     -5.18362787842,    # Joint 3: lower limit in radians (~-296 degrees)
        #     0.0,               # Joint 4: lower limit in radians
        #     0.0                # Joint 5: lower limit in radians
        # ])

        # self.joint_limit_max = np.array([
        #     5.89921287174,     # Joint 1: upper limit in radians (~338 degrees)
        #     2.70526034059,     # Joint 2: upper limit in radians (~155 degrees)
        #     0.0,               # Joint 3: upper limit in radians
        #     3.57792496659,     # Joint 4: upper limit in radians (~205 degrees)
        #     5.84685299418      # Joint 5: upper limit in radians (~334 degrees)
        # ])

        # Define joint limits (in radians)
        # self.joint_limit_min = np.array([
        #     -180 * np.pi / 180,    # Joint 1
        #     -65 * np.pi / 180,     # Joint 2
        #     -150 * np.pi / 180,    # Joint 3
        #     -102.5 * np.pi / 180,  # Joint 4
        #     -167.5 * np.pi / 180   # Joint 5
        # ])
        # self.joint_limit_max = np.array([
        #     180 * np.pi / 180,     # Joint 1
        #     90 * np.pi / 180,      # Joint 2
        #     146 * np.pi / 180,     # Joint 3
        #     102.5 * np.pi / 180,   # Joint 4
        #     167.5 * np.pi / 180    # Joint 5
        # ])

        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 5)
        self.traj_publisher = self.create_publisher(
            JointTrajectory, '/EffortJointInterface_trajectory_controller/command', 5)
        self.pose_broadcaster = TransformBroadcaster(self)

    def joint_state_callback(self, msg):
        self.current_joint_position = list(msg.position)
        current_pose = self.forward_kinematics(self.current_joint_position)
        self.broadcast_pose(current_pose)

    def broadcast_pose(self, pose):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'arm_end_effector_' + self.tf_suffix
        transform.transform.translation.x = pose[0, 3]
        transform.transform.translation.y = pose[1, 3]
        transform.transform.translation.z = pose[2, 3]
        transform.transform.rotation = rotmat2q(pose[:3, :3])
        self.pose_broadcaster.sendTransform(transform)

    def forward_kinematics(self, joint_readings, up_to_joint=5):
        T = np.eye(4)
        for i in range(up_to_joint):
            a, alpha, d, theta = (self.dh_params['a'][i],
                                  self.dh_params['alpha'][i],
                                  self.dh_params['d'][i],
                                  self.dh_params['theta'][i] + joint_readings[i])
            T_i = self.standard_dh(a, alpha, d, theta)
            T = np.dot(T, T_i)
        return T

    def standard_dh(self, a, alpha, d, theta):
        A = np.array([[np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
                      [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
                      [0, np.sin(alpha), np.cos(alpha), d],
                      [0, 0, 0, 1]])
        return A

    def rotmat2rodrigues(self, T):
        quaternion = rotmat2q(T[:3, :3])
        angle = 2 * np.arccos(quaternion[3])
        if angle == 0:
            axis = np.array([0, 0, 0])
        else:
            axis = quaternion[:3] / np.sin(angle / 2)
        p = np.zeros(6)
        p[:3] = T[:3, 3]
        p[3:] = axis * angle
        return p
