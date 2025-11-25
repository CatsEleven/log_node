import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/kohei/Downloads/ros2_ws/src/log_node/install/log_node'
