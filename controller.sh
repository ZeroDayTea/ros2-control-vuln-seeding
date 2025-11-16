#!/bin/bash

# Define cleanup function to kill all child processes gracefully
cleanup() {
    echo "Caught EXIT signal. Killing child processes..."
    # Get the PGID of the current script
    PGID=$(ps -o pgid= $ | grep -o '[0-9]*')

    # Send a termination signal to the entire process group
    if [ -n "$PGID" ]; then
        kill -TERM -- -"$PGID"
    fi

    wait  # Wait for all processes to terminate
    echo "Child processes terminated."
}

# Trap EXIT signal to ensure cleanup function is called
trap cleanup EXIT

echo "Starting r6bot_controller..."
source /opt/ros/jazzy/setup.bash
source install/setup.bash
# Start the ROS2 node in the background
ros2 launch ros2_control_demo_example_7 r6bot_controller.launch.py &

# Wait for all jobs to complete
wait
echo "Started r6bot_controller"