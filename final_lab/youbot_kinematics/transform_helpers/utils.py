#!/usr/bin/env python3

import numpy as np
from geometry_msgs.msg import Quaternion

def rotmat2q(R: np.ndarray) -> Quaternion:
    """
    Convert a 3x3 rotation matrix (R) into a ROS geometry_msgs/Quaternion.
    Uses the standard approach:
       w = 0.5 * sqrt(1 + R[0,0] + R[1,1] + R[2,2])
       x = ...
       y = ...
       z = ...
    """
    q = Quaternion()

    if R.shape != (3, 3):
        raise ValueError("Rotation matrix must be 3x3.")

    trace = R[0, 0] + R[1, 1] + R[2, 2]

    if trace > 0.0:
        S = 2.0 * np.sqrt(1.0 + trace)
        q.w = 0.25 * S
        q.x = (R[2, 1] - R[1, 2]) / S
        q.y = (R[0, 2] - R[2, 0]) / S
        q.z = (R[1, 0] - R[0, 1]) / S
    else:
        # find biggest diagonal
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            S = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            q.w = (R[2, 1] - R[1, 2]) / S
            q.x = 0.25 * S
            q.y = (R[0, 1] + R[1, 0]) / S
            q.z = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            q.w = (R[0, 2] - R[2, 0]) / S
            q.x = (R[0, 1] + R[1, 0]) / S
            q.y = 0.25 * S
            q.z = (R[1, 2] + R[2, 1]) / S
        else:
            S = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            q.w = (R[1, 0] - R[0, 1]) / S
            q.x = (R[0, 2] + R[2, 0]) / S
            q.y = (R[1, 2] + R[2, 1]) / S
            q.z = 0.25 * S

    return q
