import rclpy
import time
import threading

import numpy as np
from youbot_kinematics.youbotKineBase import YoubotKinematicBase
from youbot_kinematics.target_data import TARGET_JOINT_POSITIONS


class YoubotKinematicStudent(YoubotKinematicBase):
    def __init__(self):
        super(YoubotKinematicStudent, self).__init__(tf_suffix='student')

        youbot_joint_offsets = [170.0 * np.pi / 180.0,
                                -65.0 * np.pi / 180.0,
                                146 * np.pi / 180,
                                -102.5 * np.pi / 180,
                                -167.5 * np.pi / 180]
        # youbot_joint_offsets = [
        #     2.96705972839,    # Joint 1 offset (~170 degrees)
        #     -1.1344640138,    # Joint 2 offset (~-65 degrees)
        #     2.54818070791,    # Joint 3 offset (~146 degrees)
        #     -1.78896248329,   # Joint 4 offset (~-102.5 degrees)
        #     2.92342649709     # Joint 5 offset (~167.5 degrees)
        # ]

        self.dh_params['theta'] = [theta + offset for theta, offset in
                                   zip(self.dh_params['theta'], youbot_joint_offsets)]

        self.youbot_joint_readings_polarity = [-1, 1, 1, 1, 1]
        # self.youbot_joint_readings_polarity = [
        #     -1,  # Joint 1
        #     1,  # Joint 2
        #     1,  # Joint 3
        #     1,  # Joint 4
        #     -1   # Joint 5
        # ]

    def forward_kinematics(self, joints_readings, up_to_joint=5):
        T = np.identity(4)
        joints_readings = [sign * angle for sign, angle in zip(self.youbot_joint_readings_polarity, joints_readings)]

        for i in range(up_to_joint):
            A = self.standard_dh(self.dh_params['a'][i],
                                 self.dh_params['alpha'][i],
                                 self.dh_params['d'][i],
                                 self.dh_params['theta'][i] + joints_readings[i])
            T = T.dot(A)

        assert isinstance(T, np.ndarray), "Output wasn't of type ndarray"
        assert T.shape == (4, 4), "Output had wrong dimensions"
        return T

    def get_jacobian(self, joint):
        """
        Computes the Jacobian matrix for the given joint angles using the Standard DH convention.
        
        Args:
            joint (list or np.ndarray): Current joint angles [q1, q2, q3, q4, q5].
        
        Returns:
            np.ndarray: Jacobian matrix (6x5).
        """
        # Validate input
        if not isinstance(joint, (list, np.ndarray)) or len(joint) != 5:
            raise ValueError("Joint angles must be a list or numpy array of length 5.")
        
        # Apply joint polarities
        adjusted_joint = [joint[i] * self.youbot_joint_readings_polarity[i] for i in range(5)]
        
        # Initialize Transformation Matrix as Identity
        T = np.eye(4)
        
        # Initialize lists for positions and Z-axes
        positions = [T[:3, 3]]
        z_vectors = [np.array([0, 0, -1])]  # Corrected to match Joint 1's polarity
        
        # Compute transformations for each joint
        for i in range(5):
            A = self.standard_dh(
                self.dh_params['a'][i],
                self.dh_params['alpha'][i],
                self.dh_params['d'][i],
                self.dh_params['theta'][i] + adjusted_joint[i]
            )
            T = T @ A
            positions.append(T[:3, 3])
            z_vectors.append(T[:3, 2])
        
        # Compute Jacobian
        o_n = positions[-1]
        J = np.zeros((6, 5))
        for i in range(5):
            J_v = np.cross(z_vectors[i], o_n - positions[i])
            J_w = z_vectors[i]
            J[:3, i] = J_v
            J[3:, i] = J_w

        #J[np.abs(J) < 1e-8] = 0.00000000e+00

        return J

    def check_singularity(self, joint):
        assert isinstance(joint, list)
        assert len(joint) == 5

        jacobian = self.get_jacobian(joint)
        singularity = np.linalg.matrix_rank(jacobian) < 5
        
        #assert isinstance(singularity, bool)
        return singularity

def main(args=None):
    rclpy.init(args=args)
    kinematic_student = YoubotKinematicStudent()

    for i in range(TARGET_JOINT_POSITIONS.shape[0]):
        target_joint_angles = TARGET_JOINT_POSITIONS[i]
        target_joint_angles = target_joint_angles.tolist()
        pose = kinematic_student.forward_kinematics(target_joint_angles)
        jacobian = kinematic_student.get_jacobian(target_joint_angles)
        singularity = kinematic_student.check_singularity(target_joint_angles)

        print("Target joint angles:")
        print(target_joint_angles)
        print("Pose:")
        print(pose)
        print("Jacobian:")
        print(jacobian)
        print("Singularity:")
        print(singularity)

    rclpy.spin(kinematic_student)

    kinematic_student.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()