import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/reike/Desktop/scrivania/robotics/simulations&notebooks/infinity-autopilot-simulation/ros_ws_infinity_autopilot/install/uav_sim_ros'
