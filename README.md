# ROS2 Controller Vuln Seeding

Automated program repair of vulnerabilities in a ROS2 Controller

## Setup

Must be on Ubuntu 22.04 or higher for this to work. Has only been tested on Ubuntu 24.04.
```
./install_ros2_jazzy.sh
./build.sh
```

## Running ROS2 In A Workspace
```
source ./source_workspace.sh
\[any 'ros2' commands\]
```

## Running the Controller

```
./controller.sh
```

## Sending the Trajectory

```
./send_trajectory.sh
```
