import rclpy
from rclpy.node import Node
from scipy.linalg import expm, logm
from itertools import permutations
import time
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker
from sensor_msgs.msg import JointState

import numpy as np
from youbot_kinematics.youbotKineStudent import YoubotKinematicStudent
from youbot_kinematics.target_data import TARGET_JOINT_POSITIONS


class YoubotTrajectoryPlanning(Node):
    def __init__(self):
        # Initialize node
        super().__init__('youbot_trajectory_planner')

        # Initialize kinematic model
        self.kdl_youbot = YoubotKinematicStudent()

        # Initialize current joint positions
        self.current_joint_position = self.kdl_youbot.current_joint_position.copy()

        # Create publishers
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/EffortJointInterface_trajectory_controller/command',
            5
        )
        self.checkpoint_pub = self.create_publisher(
            Marker,
            "checkpoint_positions",
            100
        )

        # Create a subscriber to /joint_states to keep track of current joint positions
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.get_logger().info("Youbot Trajectory Planner Initialized.")

    def joint_state_callback(self, msg):
        """
        Callback function to update current joint positions from /joint_states topic.
        """
        try:
            # Define the expected order of joints
            joint_order = ["arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"]
            self.current_joint_position = np.array([msg.position[msg.name.index(joint)] for joint in joint_order])
            self.kdl_youbot.current_joint_position = self.current_joint_position.copy()
            self.get_logger().debug(f"Updated current_joint_position: {self.current_joint_position}")
        except ValueError as e:
            self.get_logger().error(f"JointState callback error: {e}")

    def run(self):
        """
        Main execution method.
        """
        self.get_logger().info("Waiting 5 seconds for everything to load up.")
        time.sleep(5.0)  # Wait to ensure all components are ready
        traj = self.q6()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = ["arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"]
        self.traj_pub.publish(traj)
        self.get_logger().info("Trajectory published.")

    def q6(self):
        """
        Main Q6 function to generate the trajectory.
        Steps:
            1. Load targets.
            2. Compute the shortest path.
            3. Generate intermediate checkpoints.
            4. Publish checkpoints for visualization.
            5. Solve IK for all checkpoints.
            6. Create and return JointTrajectory message.
        Returns:
            traj (JointTrajectory): The generated joint trajectory.
        """
        # Step 1: Load targets
        target_cart_tf, target_joint_positions = self.load_targets()

        # Step 2: Compute the shortest path
        sorted_order, min_dist = self.get_shortest_path(target_cart_tf)

        # Step 3: Generate intermediate checkpoints
        num_intermediate_points = 20  # Can be adjusted as needed
        full_checkpoint_tfs = self.intermediate_tfs(sorted_order, target_cart_tf, num_intermediate_points)

        # Step 4: Publish checkpoints for visualization
        self.publish_traj_tfs(full_checkpoint_tfs)

        # Step 5: Solve IK for all checkpoints
        init_joint_position = self.current_joint_position
        q_checkpoints = self.full_checkpoints_to_joints(full_checkpoint_tfs, init_joint_position)

        # Logging for debugging
        self.get_logger().info(f"Full checkpoint transforms shape: {full_checkpoint_tfs.shape}")
        self.get_logger().info(f"Joint checkpoints shape: {q_checkpoints.shape}")

        # Step 6: Create JointTrajectory message
        traj = JointTrajectory()
        traj.joint_names = ["arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"]
        traj.points = [
            JointTrajectoryPoint(
                positions=q_checkpoints[:, i].tolist(),
                time_from_start=rclpy.time.Duration(seconds=i * 0.5).to_msg()
            )
            for i in range(q_checkpoints.shape[1])
        ]

        assert isinstance(traj, JointTrajectory)
        return traj

    def load_targets(self):
        """
        Loads target joint positions and computes their corresponding Cartesian transforms.
        Returns:
            target_cart_tf (np.ndarray): 4x4x(n+1) array of homogeneous transforms.
            target_joint_positions (np.ndarray): 5x(n+1) array of joint positions.
        """
        num_target_positions = TARGET_JOINT_POSITIONS.shape[0]
        self.get_logger().info(f"Loading {num_target_positions} target positions.")

        # Initialize arrays
        target_joint_positions = np.zeros((5, num_target_positions + 1))
        target_cart_tf = np.repeat(
            np.identity(4).reshape(4, 4, 1),
            num_target_positions + 1,
            axis=2
        )

        # Step 1: Set initial joint position and corresponding Cartesian transform
        target_joint_positions[:, 0] = self.current_joint_position
        target_cart_tf[:, :, 0] = self.kdl_youbot.forward_kinematics(target_joint_positions[:, 0].tolist())

        # Step 2: Populate joint positions and compute forward kinematics
        for i in range(1, num_target_positions + 1):
            target_joint_positions[:, i] = TARGET_JOINT_POSITIONS[i - 1]
            fk = self.kdl_youbot.forward_kinematics(target_joint_positions[:, i].tolist())
            target_cart_tf[:, :, i] = fk
            self.get_logger().debug(f"Checkpoint {i}: Joint Positions = {target_joint_positions[:, i]}, FK =\n{fk}")

        self.get_logger().info(f"Target Cartesian transforms shape: {target_cart_tf.shape}")
        assert isinstance(target_cart_tf, np.ndarray)
        assert target_cart_tf.shape == (4, 4, num_target_positions + 1)
        assert isinstance(target_joint_positions, np.ndarray)
        assert target_joint_positions.shape == (5, num_target_positions + 1)

        return target_cart_tf, target_joint_positions

    def get_shortest_path(self, checkpoints_tf):
        """
        Computes the order of visiting checkpoints that results in the shortest total path length.
        Args:
            checkpoints_tf (np.ndarray): 4x4xn array of homogeneous transforms.
        Returns:
            sorted_order (np.ndarray): Array indicating the order of checkpoints.
            min_dist (float): Total distance of the shortest path.
        """
        num_checkpoints = checkpoints_tf.shape[2]
        if num_checkpoints <= 1:
            self.get_logger().info("Only one checkpoint provided. No path computation needed.")
            return np.array([0]), 0.0

        indices = list(range(1, num_checkpoints))  # Exclude the starting point at index 0
        min_dist = float('inf')
        best_order = None

        self.get_logger().info("Computing the shortest path through checkpoints.")

        # Generate all possible permutations of the target checkpoints
        for perm in permutations(indices):
            # Start from the initial checkpoint
            current_order = (0,) + perm
            dist = 0.0
            for i in range(len(current_order) - 1):
                a_idx = current_order[i]
                b_idx = current_order[i + 1]
                a_pos = checkpoints_tf[:3, 3, a_idx]
                b_pos = checkpoints_tf[:3, 3, b_idx]
                dist += np.linalg.norm(b_pos - a_pos)
            if dist < min_dist:
                min_dist = dist
                best_order = current_order

        sorted_order = np.array(best_order)
        self.get_logger().info(f"Shortest path distance: {min_dist:.2f}")
        self.get_logger().info(f"Sorted checkpoint order: {sorted_order}")

        assert isinstance(sorted_order, np.ndarray)
        assert sorted_order.shape == (num_checkpoints,)
        assert isinstance(min_dist, float)

        return sorted_order, min_dist

    def publish_traj_tfs(self, tfs):
        """
        Publishes the trajectory transforms as markers for visualization.
        Args:
            tfs (np.ndarray): 4x4xn array of homogeneous transforms.
        """
        id = 0
        for i in range(tfs.shape[2]):
            marker = Marker()
            marker.id = id
            id += 1
            marker.header.frame_id = 'base_link'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.0
            # Ensure color values stay within [0, 1]
            marker.color.g = min(1.0, 0.05 * id)
            marker.color.b = max(0.0, 1.0 - 0.05 * id)
            marker.pose.orientation.w = 1.0
            # Corrected position extraction
            marker.pose.position.x = tfs[0, 3, i]
            marker.pose.position.y = tfs[1, 3, i]
            marker.pose.position.z = tfs[2, 3, i]
            self.checkpoint_pub.publish(marker)
            self.get_logger().debug(f"Published marker {id} at position ({marker.pose.position.x}, "
                                     f"{marker.pose.position.y}, {marker.pose.position.z})")

    def intermediate_tfs(self, sorted_checkpoint_idx, target_checkpoint_tfs, num_points):
        """
        Generates intermediate transforms between sorted checkpoints.
        Args:
            sorted_checkpoint_idx (np.ndarray): Array indicating the order of checkpoints.
            target_checkpoint_tfs (np.ndarray): 4x4xn array of homogeneous transforms.
            num_points (int): Number of intermediate points between checkpoints.
        Returns:
            full_checkpoint_tfs (np.ndarray): 4x4xm array of homogeneous transforms.
        """
        self.get_logger().info("Generating intermediate transforms.")
        full_tfs = []

        for i in range(len(sorted_checkpoint_idx) - 1):
            a_idx = sorted_checkpoint_idx[i]
            b_idx = sorted_checkpoint_idx[i + 1]
            a_tf = target_checkpoint_tfs[:, :, a_idx]
            b_tf = target_checkpoint_tfs[:, :, b_idx]
            intermediate = self.decoupled_rot_and_trans(a_tf, b_tf, num_points)
            full_tfs.append(intermediate)

        # Concatenate all intermediate transforms
        full_checkpoint_tfs = np.concatenate(full_tfs, axis=2)
        self.get_logger().info(f"Full checkpoint transforms generated with shape: {full_checkpoint_tfs.shape}")
        return full_checkpoint_tfs

    def decoupled_rot_and_trans(self, checkpoint_a_tf, checkpoint_b_tf, num_points):
        """
        Interpolates between two transforms by decoupling rotation and translation.
        Args:
            checkpoint_a_tf (np.ndarray): 4x4 transform of checkpoint A.
            checkpoint_b_tf (np.ndarray): 4x4 transform of checkpoint B.
            num_points (int): Number of intermediate points.
        Returns:
            tfs (np.ndarray): 4x4xnum_points array of interpolated transforms.
        """
        self.get_logger().info("Interpolating between two checkpoints.")

        # Extract rotation and translation
        A_rot = checkpoint_a_tf[:3, :3]
        B_rot = checkpoint_b_tf[:3, :3]
        A_trans = checkpoint_a_tf[:3, 3]
        B_trans = checkpoint_b_tf[:3, 3]

        # Interpolate translations linearly
        translations = np.linspace(A_trans, B_trans, num_points)

        # Compute relative rotation from A to B
        relative_rot = np.dot(A_rot.T, B_rot)
        log_rel_rot = logm(relative_rot)

        # Handle cases where logm might return complex numbers due to numerical errors
        if np.iscomplexobj(log_rel_rot):
            log_rel_rot = np.real(log_rel_rot)

        # Interpolate rotations
        rotations = [np.dot(A_rot, expm(t * log_rel_rot)) for t in np.linspace(0, 1, num_points)]

        # Construct homogeneous transforms
        tfs = np.zeros((4, 4, num_points))
        for i in range(num_points):
            tfs[:3, :3, i] = rotations[i]
            tfs[:3, 3, i] = translations[i]
            tfs[3, 3, i] = 1.0

        self.get_logger().debug(f"Generated {num_points} intermediate transforms between checkpoints.")
        return tfs

    def full_checkpoints_to_joints(self, full_checkpoint_tfs, init_joint_position):
        """
        Solves inverse kinematics for all checkpoint transforms to obtain joint positions.
        Args:
            full_checkpoint_tfs (np.ndarray): 4x4xn array of homogeneous transforms.
            init_joint_position (np.ndarray): Initial joint positions (5,).
        Returns:
            q_checkpoints (np.ndarray): 5xn array of joint positions.
        """
        num_checkpoints = full_checkpoint_tfs.shape[2]
        q_checkpoints = np.zeros((5, num_checkpoints))
        q_checkpoints[:, 0] = init_joint_position

        self.get_logger().info("Solving inverse kinematics for all checkpoints.")

        for i in range(1, num_checkpoints):
            desired_pose = full_checkpoint_tfs[:, :, i]
            q0 = q_checkpoints[:, i - 1]
            q, error = self.ik_position_only(desired_pose, q0)

            if error > 1e-3:
                self.get_logger().warn(f"IK did not converge for checkpoint {i}, error: {error:.4f}")
            else:
                self.get_logger().info(f"IK converged for checkpoint {i} with error: {error:.4f}")

            q_checkpoints[:, i] = q

        self.get_logger().info(f"All IK solutions computed with shape: {q_checkpoints.shape}")
        return q_checkpoints


    # def ik_position_only(self, pose, q0, lam_initial=1.0, lam_min=0.25, damping_initial=0.1, damping_min=0.001, 
    #                     num=500, tol=1e-3, delta_q_max=np.deg2rad(5)):
    #     """
    #     Performs position-only inverse kinematics using the damped least squares method with improvements for smaller errors.
        
    #     Args:
    #         pose (np.ndarray): Desired 4x4 homogeneous transform.
    #         q0 (np.ndarray): Initial joint positions (5,).
    #         lam_initial (float): Initial step size scaling factor.
    #         lam_min (float): Minimum step size scaling factor.
    #         damping_initial (float): Initial damping factor.
    #         damping_min (float): Minimum damping factor.
    #         num (int): Maximum number of iterations.
    #         tol (float): Tolerance for convergence based on error norm and error reduction.
    #         delta_q_max (np.ndarray): Maximum allowable change in joint angles per iteration (radians).
            
    #     Returns:
    #         q (np.ndarray): Solved joint positions (5,).
    #         error_norm (float): Final position error.
    #     """
    #     q = q0.copy()
    #     damping = damping_initial  # Initialize damping
    #     lam = lam_initial        # Initialize learning rate
    #     previous_error_norm = np.inf

    #     for iter_num in range(num):
    #         # Compute forward kinematics
    #         fk = self.kdl_youbot.forward_kinematics(q.tolist())
    #         current_pos = fk[:3, 3]
    #         desired_pos = pose[:3, 3]
    #         position_error = desired_pos - current_pos
    #         error_norm = np.linalg.norm(position_error)

    #         # Check for convergence based on error norm
    #         if error_norm < tol:
    #             self.get_logger().debug(f"IK converged in {iter_num} iterations with error {error_norm:.6f}.")
    #             break

    #         # Compute change in error
    #         delta_error = previous_error_norm - error_norm
    #         if delta_error < tol:
    #             self.get_logger().debug(f"Minimal error reduction at iteration {iter_num}. Converging.")
    #             break
    #         previous_error_norm = error_norm

    #         # Compute Jacobian
    #         jacobian = self.kdl_youbot.get_jacobian(q.tolist())  # Assuming shape (6,5)
    #         J_pos = jacobian[:3, :]  # Position part of the Jacobian (3,5)

    #         # Compute the damped pseudo-inverse using SVD
    #         U, S, Vt = np.linalg.svd(J_pos, full_matrices=False)
    #         S_damped = S / (S**2 + damping**2)
    #         J_pos_pinv = Vt.T @ np.diag(S_damped) @ U.T  # (5,3)

    #         # Compute change in joint angles
    #         delta_q = lam * (J_pos_pinv @ position_error)  # (5,)

    #         # Clamp delta_q to maximum joint angle change
    #         delta_q = np.clip(delta_q, -delta_q_max, delta_q_max)

    #         # Update joint angles with polarity consideration
    #         delta_q = self.kdl_youbot.youbot_joint_readings_polarity * delta_q
    #         q += delta_q

    #         # Enforce joint limits
    #         q = np.clip(q, self.kdl_youbot.joint_limit_min, self.kdl_youbot.joint_limit_max)

    #         # Decay the learning rate
    #         lam = max(lam * 0.99, lam_min)

    #         # Adaptive damping: Decrease damping as iterations increase
    #         damping = max(damping * 0.99, damping_min)

    #         # Optional: Log progress every 50 iterations
    #         if iter_num % 50 == 0:
    #             self.get_logger().debug(f"Iteration {iter_num}: Error = {error_norm:.6f}, Damping = {damping:.6f}, Lambda = {lam:.6f}")

    #     else:
    #         self.get_logger().warn(f"IK did not converge within {num} iterations. Final error: {error_norm:.6f}")

    #     return q, error_norm

    def ik_position_only(self, pose, q0, lam=0.25, num=500):
        """This function implements position-only inverse kinematics.
        
        Args:
            pose (np.ndarray, 4x4): 4x4 transformations describing the pose of the end-effector position.
            q0 (np.ndarray, 5x1): A 5x1 array for the initial starting point of the algorithm.
            lam (float): A damping factor for the optimization steps.
            num (int): Maximum number of iterations for the algorithm.
        
        Returns:
            q (np.ndarray, 5x1): The IK solution for the given pose.
            error (float): The Cartesian error of the solution.
        """
        # Extract the desired position from the pose
        desired_position = pose[:3, 3]
        
        # Initialize joint positions with the starting point
        q = q0.copy()
        
        # Damping factor for the pseudo-inverse computation
        damping = 1e-3
        
        for i in range(num):
            # Compute forward kinematics to get the current end-effector pose
            fk = self.kdl_youbot.forward_kinematics(q.tolist())
            current_position = fk[:3, 3]
            
            # Calculate the position error
            position_error = desired_position - current_position
            error_norm = np.linalg.norm(position_error)
            
            # Break if the error is small enough
            if error_norm < 1e-4:
                break
            
            # Compute the Jacobian and extract its position part
            jacobian = self.kdl_youbot.get_jacobian(q.tolist())
            J_pos = jacobian[:3, :]  # Only take the position part (3x5)
            
            # Compute the damped pseudo-inverse of the Jacobian
            JT = J_pos.T
            JJT = J_pos @ JT
            identity = np.eye(JJT.shape[0])
            inv_term = np.linalg.inv(JJT + damping * identity)
            J_pos_pinv = JT @ inv_term
            
            # Compute the change in joint positions
            delta_q = lam * J_pos_pinv @ position_error
            
            # Update the joint positions
            q += delta_q
            
            # Ensure the joint positions are within limits (optional)
            # q = np.clip(q, self.joint_limit_min, self.joint_limit_max)
        
        else:
            # If it reaches the maximum number of iterations
            self.get_logger().warn(f"IK did not converge within {num} iterations. Final error: {error_norm:.6f}")
        
        return q, error_norm


    def publish_traj_tfs(self, tfs):
        """
        Publishes the trajectory transforms as markers for visualization.
        Args:
            tfs (np.ndarray): 4x4xn array of homogeneous transforms.
        """
        id = 0
        for i in range(tfs.shape[2]):
            marker = Marker()
            marker.id = id
            id += 1
            marker.header.frame_id = 'base_link'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.0
            # Ensure color values stay within [0, 1]
            marker.color.g = min(1.0, 0.05 * id)
            marker.color.b = max(0.0, 1.0 - 0.05 * id)
            marker.pose.orientation.w = 1.0
            # Corrected position extraction
            marker.pose.position.x = tfs[0, 3, i]
            marker.pose.position.y = tfs[1, 3, i]
            marker.pose.position.z = tfs[2, 3, i]
            self.checkpoint_pub.publish(marker)
            self.get_logger().debug(f"Published marker {id} at position ({marker.pose.position.x}, "
                                     f"{marker.pose.position.y}, {marker.pose.position.z})")

    def intermediate_tfs(self, sorted_checkpoint_idx, target_checkpoint_tfs, num_points):
        """
        Generates intermediate transforms between sorted checkpoints.
        Args:
            sorted_checkpoint_idx (np.ndarray): Array indicating the order of checkpoints.
            target_checkpoint_tfs (np.ndarray): 4x4xn array of homogeneous transforms.
            num_points (int): Number of intermediate points between checkpoints.
        Returns:
            full_checkpoint_tfs (np.ndarray): 4x4xm array of homogeneous transforms.
        """
        self.get_logger().info("Generating intermediate transforms.")
        full_tfs = []

        for i in range(len(sorted_checkpoint_idx) - 1):
            a_idx = sorted_checkpoint_idx[i]
            b_idx = sorted_checkpoint_idx[i + 1]
            a_tf = target_checkpoint_tfs[:, :, a_idx]
            b_tf = target_checkpoint_tfs[:, :, b_idx]
            intermediate = self.decoupled_rot_and_trans(a_tf, b_tf, num_points)
            full_tfs.append(intermediate)

        # Concatenate all intermediate transforms
        full_checkpoint_tfs = np.concatenate(full_tfs, axis=2)
        self.get_logger().info(f"Full checkpoint transforms generated with shape: {full_checkpoint_tfs.shape}")
        return full_checkpoint_tfs

    def decoupled_rot_and_trans(self, checkpoint_a_tf, checkpoint_b_tf, num_points):
        """
        Interpolates between two transforms by decoupling rotation and translation.
        Args:
            checkpoint_a_tf (np.ndarray): 4x4 transform of checkpoint A.
            checkpoint_b_tf (np.ndarray): 4x4 transform of checkpoint B.
            num_points (int): Number of intermediate points.
        Returns:
            tfs (np.ndarray): 4x4xnum_points array of interpolated transforms.
        """
        self.get_logger().info("Interpolating between two checkpoints.")

        # Extract rotation and translation
        A_rot = checkpoint_a_tf[:3, :3]
        B_rot = checkpoint_b_tf[:3, :3]
        A_trans = checkpoint_a_tf[:3, 3]
        B_trans = checkpoint_b_tf[:3, 3]

        # Interpolate translations linearly
        translations = np.linspace(A_trans, B_trans, num_points)

        # Compute relative rotation from A to B
        relative_rot = np.dot(A_rot.T, B_rot)
        log_rel_rot = logm(relative_rot)

        # Handle cases where logm might return complex numbers due to numerical errors
        if np.iscomplexobj(log_rel_rot):
            log_rel_rot = np.real(log_rel_rot)

        # Interpolate rotations
        rotations = [np.dot(A_rot, expm(t * log_rel_rot)) for t in np.linspace(0, 1, num_points)]

        # Construct homogeneous transforms
        tfs = np.zeros((4, 4, num_points))
        for i in range(num_points):
            tfs[:3, :3, i] = rotations[i]
            tfs[:3, 3, i] = translations[i]
            tfs[3, 3, i] = 1.0

        self.get_logger().debug(f"Generated {num_points} intermediate transforms between checkpoints.")
        return tfs


def main(args=None):
    rclpy.init(args=args)

    youbot_planner = YoubotTrajectoryPlanning()

    try:
        youbot_planner.run()
        rclpy.spin(youbot_planner)
    except KeyboardInterrupt:
        youbot_planner.get_logger().info("Shutting down Youbot Trajectory Planner.")
    finally:
        youbot_planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
