import rclpy
from rclpy.node import Node
from scipy.linalg import expm
from scipy.linalg import logm
from itertools import permutations
import time
import threading
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker

import numpy as np
from youbot_kinematics.youbotKineStudent import YoubotKinematicStudent
from youbot_kinematics.target_data import TARGET_JOINT_POSITIONS


class YoubotTrajectoryPlanning(Node):
    def __init__(self):
        super().__init__('youbot_trajectory_planner')
        self.kdl_youbot = YoubotKinematicStudent()
        self.traj_pub = self.create_publisher(JointTrajectory, '/EffortJointInterface_trajectory_controller/command', 5)
        self.checkpoint_pub = self.create_publisher(Marker, "checkpoint_positions", 100)

    def run(self):
        self.get_logger().info("Waiting 5 seconds for everything to load up.")
        time.sleep(2.0)
        traj = self.q6()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = ["arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"]
        self.traj_pub.publish(traj)

    def q6(self):
        target_cart_tf, target_joint_positions = self.load_targets()
        sorted_order, _ = self.get_shortest_path(target_cart_tf)
        full_checkpoint_tfs = self.intermediate_tfs(sorted_order, target_cart_tf, num_points=10)
        self.publish_traj_tfs(full_checkpoint_tfs)
        init_joint_position = self.kdl_youbot.current_joint_position
        q_checkpoints = self.full_checkpoints_to_joints(full_checkpoint_tfs, init_joint_position)

        self.get_logger().info(f"full_checkpoint_tfs: {full_checkpoint_tfs}")
        self.get_logger().info(f"q_checkpoints: {q_checkpoints}")

        traj = JointTrajectory()
        traj.points = [JointTrajectoryPoint(positions=q, time_from_start=rclpy.time.Duration(seconds=i * 0.5).to_msg())
                       for i, q in enumerate(q_checkpoints.T)]
        return traj

    def load_targets(self):
        num_target_positions = TARGET_JOINT_POSITIONS.shape[0]
        target_joint_positions = TARGET_JOINT_POSITIONS.T
        target_cart_tf = np.zeros((4, 4, num_target_positions))
        for i in range(num_target_positions):
            target_cart_tf[:, :, i] = self.kdl_youbot.forward_kinematics(target_joint_positions[:, i].tolist())
        return target_cart_tf, target_joint_positions

    def get_shortest_path(self, checkpoints_tf):
        num_checkpoints = checkpoints_tf.shape[2]
        indices = range(num_checkpoints)
        min_dist = float('inf')
        sorted_order = None
        for perm in permutations(indices):
            dist = 0
            for i in range(len(perm) - 1):
                diff = checkpoints_tf[:3, 3, perm[i + 1]] - checkpoints_tf[:3, 3, perm[i]]
                dist += np.linalg.norm(diff)
            if dist < min_dist:
                min_dist = dist
                sorted_order = np.array(perm)
        return sorted_order, min_dist

    def publish_traj_tfs(self, tfs):
        """This function gets a np.ndarray of transforms and publishes them in a color coded fashion to show how the
        Cartesian path of the robot end-effector.
        Args:
            tfs (np.ndarray): A array of 4x4xn homogenous transformations specifying the end-effector trajectory.
        """
        id = 0
        for i in range(0, tfs.shape[2]):
            marker = Marker()
            marker.id = id
            id += 1
            marker.header.frame_id = 'base_link'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0 + id * 0.05
            marker.color.b = 1.0 - id * 0.05
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = tfs[0, -1, i]
            marker.pose.position.y = tfs[1, -1, i]
            marker.pose.position.z = tfs[2, -1, i]
            self.checkpoint_pub.publish(marker)

    # def intermediate_tfs(self, sorted_checkpoint_idx, target_checkpoint_tfs, num_points):
    #     full_tfs = []
    #     for i in range(len(sorted_checkpoint_idx) - 1):
    #         a_idx = sorted_checkpoint_idx[i]
    #         b_idx = sorted_checkpoint_idx[i + 1]
    #         tfs = self.decoupled_rot_and_trans(target_checkpoint_tfs[:, :, a_idx],
    #                                            target_checkpoint_tfs[:, :, b_idx],
    #                                            num_points)
    #         full_tfs.append(tfs)
    #     full_tfs = np.concatenate(full_tfs, axis=2)
    #     return full_tfs
    
    def intermediate_tfs(self, sorted_checkpoint_idx, target_checkpoint_tfs, num_points):
        full_tfs = []
        for i in range(len(sorted_checkpoint_idx) - 1):
            a_idx = sorted_checkpoint_idx[i]
            b_idx = sorted_checkpoint_idx[i + 1]
            tfs = self.decoupled_rot_and_trans(target_checkpoint_tfs[:, :, a_idx],
                                            target_checkpoint_tfs[:, :, b_idx],
                                            num_points)
            full_tfs.append(tfs)
        full_tfs = np.concatenate(full_tfs, axis=2)
        return full_tfs

    def decoupled_rot_and_trans(self, checkpoint_a_tf, checkpoint_b_tf, num_points):
        translations = np.linspace(checkpoint_a_tf[:3, 3], checkpoint_b_tf[:3, 3], num_points)
        rotations = [expm(t * logm(np.dot(checkpoint_a_tf[:3, :3].T, checkpoint_b_tf[:3, :3]))) for t in
                     np.linspace(0, 1, num_points)]
        tfs = np.zeros((4, 4, num_points))
        for i in range(num_points):
            tfs[:3, :3, i] = rotations[i]
            tfs[:3, 3, i] = translations[i]
            tfs[3, 3, i] = 1.0
        return tfs

    def full_checkpoints_to_joints(self, full_checkpoint_tfs, init_joint_position):
        q_checkpoints = np.zeros((5, full_checkpoint_tfs.shape[2]))
        q_checkpoints[:, 0] = init_joint_position
        for i in range(1, full_checkpoint_tfs.shape[2]):
            q, _ = self.ik_position_only(full_checkpoint_tfs[:, :, i], q_checkpoints[:, i - 1])
            q_checkpoints[:, i] = q
        return q_checkpoints

    def ik_position_only(self, pose, q0, lam=0.1, num=1000):
        q = q0.copy()
        for _ in range(num):
            fk = self.kdl_youbot.forward_kinematics(q.tolist())
            position_error = pose[:3, 3] - fk[:3, 3]
            if np.linalg.norm(position_error) < 1e-3:
                break
            jacobian = self.kdl_youbot.get_jacobian(q.tolist())
            delta_q = lam * np.dot(np.linalg.pinv(jacobian[:3, :]), position_error)
            q += delta_q

            q = np.clip(q, self.kdl_youbot.joint_limit_min, self.kdl_youbot.joint_limit_max)
        error = np.linalg.norm(position_error)
        return q, error

def main(args=None):
    rclpy.init(args=args)
    youbot_planner = YoubotTrajectoryPlanning()
    youbot_planner.run()
    rclpy.spin(youbot_planner)
    youbot_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
