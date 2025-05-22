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
[any 'ros2' commands]
```

## Running the example

```
[In one workspace]
./controller.sh
[In a separate workspace]
```

## Program repair with voter

```
[build necessary files]
./build.sh
./build_controllers.sh

[in one workspace]
./start_controllers.sh
[in a second workspace]
./controller.sh
[in a third workspace]
python3 voter.py
[in a fourth workspace]
./send_trajectory.sh

[afterwards]
./cleanup.sh
```
