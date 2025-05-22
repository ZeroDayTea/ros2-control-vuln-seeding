ROS_DISTRO=jazzy
source /opt/ros/${ROS_DISTRO}/setup.bash
source install/setup.bash
ros2 launch ros2_control_demo_example_7 send_trajectory.launch.py
