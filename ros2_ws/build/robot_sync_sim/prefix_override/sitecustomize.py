import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/roey/assaf_project/ros2_ws/install/robot_sync_sim'
