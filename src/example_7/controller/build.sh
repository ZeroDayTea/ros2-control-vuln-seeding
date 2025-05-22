#! /bin/bash

dir="/isis/home/cailm/ros2_ws/src/ros2_control_demos/example_7/controller"

# $dir/c0/build_controller.sh $dir

gcc -c -o $dir/c0/controller.o $dir/c0/controller.c
g++ -c -o $dir/c0/controller_main.o $dir/c0/controller_main.cpp
g++ -o $dir/tmp0 $dir/c0/controller.o $dir/c0/controller_main.o

gcc -c -o $dir/c1/controller.o $dir/c1/controller.c
g++ -c -o $dir/c1/controller_main.o $dir/c1/controller_main.cpp
g++ -o $dir/tmp1 $dir/c0/controller.o $dir/c1/controller_main.o

gcc -c -o $dir/c2/controller.o $dir/c2/controller.c
g++ -c -o $dir/c2/controller_main.o $dir/c2/controller_main.cpp
g++ -o $dir/tmp2 $dir/c2/controller.o $dir/c2/controller_main.o

cp $dir/tmp0 /isis/home/cailm/ros2_ws/src/ros2_control_demos/tmp0
cp $dir/tmp1 /isis/home/cailm/ros2_ws/src/ros2_control_demos/tmp1
cp $dir/tmp2 /isis/home/cailm/ros2_ws/src/ros2_control_demos/tmp2