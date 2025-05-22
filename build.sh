ROS_DISTRO=jazzy

source /opt/ros/${ROS_DISTRO}/setup.bash

echo "Installing Project Dependencies"
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro $ROS_DISTRO

echo "Building Project"
colcon build --symlink-install

echo ""
echo "Build Complete"
echo "To run in a workspace first run ./source_workspace.sh"
