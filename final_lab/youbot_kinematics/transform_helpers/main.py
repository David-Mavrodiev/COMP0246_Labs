import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np
from transform_helpers.utils import rotmat2q  # Utility function to convert rotation matrix to quaternion

BASE_FRAME = "base"
FRAMES = ["fr3_link0", "fr3_link1", "fr3_link2", "fr3_link3", "fr3_link4",
          "fr3_link5", "fr3_link6", "fr3_link7", "fr3_link8"]

# Classic DH Parameters for the Franka FR3 robot arm
a_list = [0, 0, 0, 0.0825, -0.0825, 0, 0.088, 0]
d_list = [0.333, 0, 0.316, 0, 0.384, 0, 0, 0.107]
alpha_list = [0, -np.pi / 2, np.pi / 2, np.pi / 2, -np.pi / 2, np.pi / 2, np.pi / 2, 0]
theta_list = [0] * len(alpha_list)  # Default joint angles
DH_PARAMS = np.array([a_list, d_list, alpha_list, theta_list]).T

AXES = [
    [0, 0, 1],  # Joint 1
    [0, 0, 1],  # Joint 2
    [0, 0, 1],  # Joint 3
    [0, 0, 1],  # Joint 4
    [0, 0, 1],  # Joint 5
    [0, 0, 1],  # Joint 6
    [0, 0, 1],  # Joint 7
    [0, 0, 1],  # Joint 8 (fixed, no rotation)
]

def get_transform_n_to_n_minus_one(a, d, alpha, theta, axis):
    """
    Compute the transformation matrix using DH parameters and joint axis.
    Args:
        a (float): Link length (translation along x-axis).
        d (float): Link offset (translation along z-axis).
        alpha (float): Link twist (rotation about x-axis).
        theta (float): Joint angle (rotation about the specified axis).
        axis (list): Axis of rotation [x, y, z].

    Returns:
        np.ndarray: 4x4 transformation matrix.
    """
    cos_theta, sin_theta = np.cos(theta), np.sin(theta)
    cos_alpha, sin_alpha = np.cos(alpha), np.sin(alpha)

    # Base transformation using DH parameters
    base_transform = np.array([
        [1, 0, 0, a],
        [0, cos_alpha, -sin_alpha, -sin_alpha * d],
        [0, sin_alpha, cos_alpha, cos_alpha * d],
        [0, 0, 0, 1],
    ])

    # Determine the axis and its direction
    axis = np.array(axis)
    normalized_axis = axis / np.linalg.norm(axis)  # Normalize axis
    direction = np.sign(normalized_axis)  # Determine positive or negative direction

    if np.all(direction == [1, 0, 0]) or np.all(direction == [-1, 0, 0]):  # Rotation about X-axis
        sin_theta *= direction[0]  # Adjust for negative rotation
        rotation_transform = np.array([
            [1, 0, 0, 0],
            [0, cos_theta, -sin_theta, 0],
            [0, sin_theta, cos_theta, 0],
            [0, 0, 0, 1],
        ])
    elif np.all(direction == [0, 1, 0]) or np.all(direction == [0, -1, 0]):  # Rotation about Y-axis
        sin_theta *= direction[1]  # Adjust for negative rotation
        rotation_transform = np.array([
            [cos_theta, 0, sin_theta, 0],
            [0, 1, 0, 0],
            [-sin_theta, 0, cos_theta, 0],
            [0, 0, 0, 1],
        ])
    elif np.all(direction == [0, 0, 1]) or np.all(direction == [0, 0, -1]):  # Rotation about Z-axis
        sin_theta *= direction[2]  # Adjust for negative rotation
        rotation_transform = np.array([
            [cos_theta, -sin_theta, 0, 0],
            [sin_theta, cos_theta, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ])
    else:
        raise ValueError(f"Unsupported axis: {axis}")

    return base_transform @ rotation_transform

class ForwardKinematicCalculator(Node):

    def __init__(self):
        super().__init__('fk_calculator')

        # Joint state subscription
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.publish_transforms, 10)

        # Transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.prefix = "my_robot/"

    def publish_transforms(self, msg: JointState):
        """
        Compute and publish transformations for each joint/link using the classic DH convention.
        Args:
            msg (JointState): The message containing joint positions.
        """
        self.get_logger().debug(str(msg))
        for i in range(len(FRAMES)):
            frame_id = self.prefix + FRAMES[i]
            parent_id = self.prefix + FRAMES[i - 1] if i != 0 else self.prefix + BASE_FRAME

            if i == 0:
                # Base frame transformation
                local_transform = np.eye(4)  # Identity for the base frame
            else:
                # Handle all subsequent revolute joints
                a, d, alpha, _ = DH_PARAMS[i - 1]  # Correct DH parameters
                axis = AXES[i - 1]
                theta = msg.position[i - 1] if i >= 1 and i < 8 else 0 # Use the corresponding joint angle

                local_transform = get_transform_n_to_n_minus_one(a, d, alpha, theta, axis)

            # Compute the quaternion for the rotation
            quat = rotmat2q(local_transform[:3, :3])

            # Fill the TransformStamped message
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = parent_id  # Parent frame ID
            t.child_frame_id = frame_id   # Child frame ID
            t.transform.translation.x = local_transform[0, 3]
            t.transform.translation.y = local_transform[1, 3]
            t.transform.translation.z = local_transform[2, 3]
            t.transform.rotation.x = quat.x
            t.transform.rotation.y = quat.y
            t.transform.rotation.z = quat.z
            t.transform.rotation.w = quat.w

            # Publish the transformation
            self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)

    # Initialize the node
    node = ForwardKinematicCalculator()
    rclpy.spin(node)

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
