#!/bin/bash

cleanup() {
    echo "Caught EXIT signal. Killing child processes..."
    kill 0  # Kills all processes in the current process group
    # Or: kill -- -$$ # Kills the process group leader and its children
    wait # Wait for background processes to terminate
    echo "Child processes terminated."
}

# Trap the EXIT signal to call the cleanup function
trap cleanup EXIT

echo "Starting r6bot_controller..."
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch ros2_control_demo_example_7 r6bot_controller.launch.py
echo "Started r6bot_controller"

