import rclpy
from rclpy.node import Node
from rclpy.clock import ROSClock
from trajectory_msgs.msg import JointTrajectory
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np
from youbot_kinematics.youbotKineStudent import YoubotKinematicStudent
from time import sleep


class YoubotTrajectoryFollower(Node):
    def __init__(self):
        super().__init__('youbot_trajectory_follower')

        # Create subscriber to listen to JointTrajectory messages
        self.traj_sub = self.create_subscription(
            JointTrajectory,
            '/EffortJointInterface_trajectory_controller/command',
            self.traj_callback,
            10
        )

        # Create a TransformBroadcaster to publish end-effector transforms
        self.transform_broadcaster = TransformBroadcaster(self)

        # Initialize the kinematics solver
        self.kinematics_solver = YoubotKinematicStudent()

        self.get_logger().info("YoubotTrajectoryFollower node started.")

    def traj_callback(self, msg: JointTrajectory):
        """Callback for processing received JointTrajectory messages."""
        self.get_logger().info("Received trajectory with {} points.".format(len(msg.points)))

        start_time = self.get_clock().now()

        for point in msg.points:
            # Extract joint positions from the trajectory point
            joint_positions = point.positions

            # Validate joint positions
            self.get_logger().info(f"Received joint positions: {joint_positions}")

            # Compute forward kinematics to get the end-effector transform
            fk_result = self.kinematics_solver.forward_kinematics(joint_positions)
            if not self.validate_fk(fk_result):
                self.get_logger().error(f"Invalid FK result: {fk_result}")
                continue

            transform = self.compute_end_effector_transform(fk_result)

            # Publish the transform
            self.publish_transform(transform)

            # Wait for the specified time before processing the next point
            duration = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            target_time = start_time + rclpy.time.Duration(seconds=duration)

            while self.get_clock().now() < target_time:
                rclpy.spin_once(self, timeout_sec=0.01)

    def validate_fk(self, fk_matrix):
        """Validate forward kinematics result."""
        is_orthogonal = np.allclose(
            np.dot(fk_matrix[:3, :3], fk_matrix[:3, :3].T), np.eye(3), atol=1e-3
        )
        is_finite = np.isfinite(fk_matrix).all()
        return is_orthogonal and is_finite

    def compute_end_effector_transform(self, fk_matrix):
        """Compute the end-effector transform using forward kinematics."""
        transform = TransformStamped()

        # Populate the transform message
        transform.header.stamp = ROSClock().now().to_msg()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = 'end_effector'

        transform.transform.translation.x = fk_matrix[0, 3]
        transform.transform.translation.y = fk_matrix[1, 3]
        transform.transform.translation.z = fk_matrix[2, 3]

        # Extract rotation as a quaternion
        rot_matrix = fk_matrix[:3, :3]
        quaternion = self.rotmat2q(rot_matrix)
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]

        self.get_logger().info(
            f"Generated Transform:\nTranslation: ({transform.transform.translation.x}, {transform.transform.translation.y}, {transform.transform.translation.z})\n"
            f"Rotation (quaternion): ({quaternion[0]}, {quaternion[1]}, {quaternion[2]}, {quaternion[3]})"
        )
        return transform

    def rotmat2q(self, rot_matrix):
        """Convert rotation matrix to quaternion."""
        # Ensure the rotation matrix is orthogonal
        u, _, vh = np.linalg.svd(rot_matrix)
        rot_matrix = np.dot(u, vh)

        # Compute quaternion
        qw = np.sqrt(1 + rot_matrix[0, 0] + rot_matrix[1, 1] + rot_matrix[2, 2]) / 2
        qx = (rot_matrix[2, 1] - rot_matrix[1, 2]) / (4 * qw)
        qy = (rot_matrix[0, 2] - rot_matrix[2, 0]) / (4 * qw)
        qz = (rot_matrix[1, 0] - rot_matrix[0, 1]) / (4 * qw)
        return [qx, qy, qz, qw]

    def publish_transform(self, transform):
        """Publish the computed transform."""
        self.get_logger().info(f"Broadcasting transform: {transform}")
        self.transform_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = YoubotTrajectoryFollower()
    rclpy.spin(node)

    # Clean up
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
