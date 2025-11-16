#!/bin/bash

# Define cleanup function
cleanup() {
    echo "Caught EXIT signal. Killing child processes..."
    kill -- -$  # Kills all processes in the process group of the script
    wait  # Wait for background processes to terminate
    echo "Child processes terminated."
}

# Trap the EXIT signal to call the cleanup function
trap cleanup EXIT

# Ensure the current script runs in its own process group
set -m

echo "Starting r6bot_controller..."
# Source ROS environment
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Start the ROS2 launch process in the background to better manage it
ros2 launch ros2_control_demo_example_7 r6bot_controller.launch.py &

# Wait for the ROS2 process to prevent the script from exiting immediately
wait
echo "Started r6bot_controller"