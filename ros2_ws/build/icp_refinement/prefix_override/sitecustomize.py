import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/aom/Documents/GitHub/Mobile_Lab1/ros2_ws/install/icp_refinement'
