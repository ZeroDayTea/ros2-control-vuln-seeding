# ROS2 Controller Vuln Seeding

Automated program repair of vulnerabilities in a ROS2 Controller

## Setup

Must be on Ubuntu 22.04 or higher for this to work. Has only been tested on Ubuntu 24.04.
```
./install_ros2_jazzy.sh
./build.sh
```

## Running the Controller

```
./source_workspace.sh
./controller.sh
```

## Sending the Trajectory

```
./source_workspace.sh
./send_trajectory.sh
```
