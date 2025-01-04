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

        self.dh_params['theta'] = [theta + offset for theta, offset in
                                   zip(self.dh_params['theta'], youbot_joint_offsets)]

        self.youbot_joint_readings_polarity = [-1, 1, 1, 1, 1]

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
        assert isinstance(joint, list)
        assert len(joint) == 5

        z = np.array([0, 0, -1])  # KDL Jacobian uses this convention
        J = np.zeros((6, 5))

        T = np.eye(4)
        positions = []
        z_vectors = [np.array([0, 0, 1])]

        for i in range(5):
            A = self.standard_dh(self.dh_params['a'][i],
                                 self.dh_params['alpha'][i],
                                 self.dh_params['d'][i],
                                 self.dh_params['theta'][i] + joint[i])
            T = T.dot(A)
            positions.append(T[:3, 3])
            z_vectors.append(T[:3, 2])

        for i in range(5):
            J[:3, i] = np.cross(z_vectors[i], positions[-1] - positions[i])
            J[3:, i] = z_vectors[i]

        jacobian = J
        assert jacobian.shape == (6, 5)
        return jacobian

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
