import sys
if sys.prefix == '/Users/davidm/miniforge3/envs/roboenv-py3.10':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/Users/davidm/Documents/repos/ucl/COMP0246_Labs/lab1submission/ros2_ws/install/youbot_kinematics'
