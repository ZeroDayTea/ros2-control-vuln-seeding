#!/bin/bash

cleanup() {
    echo "Caught EXIT signal. Killing child processes..."
    # Ensure kill command targets the entire process group
    pkill -TERM -P $  # Kills all child processes under the script's process group
    wait  # Wait for all processes to terminate
    echo "Child processes terminated."
}

# Trap EXIT signal to call the cleanup function
trap cleanup EXIT

# Start your ROS environment setup
echo "Starting r6bot_controller..."
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Run the main ROS2 launch command in the background
ros2 launch ros2_control_demo_example_7 r6bot_controller.launch.py &

# Wait for all background jobs to complete before finishing script execution
wait
echo "Started r6bot_controller"