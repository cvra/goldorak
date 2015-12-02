cmake_minimum_required(VERSION 2.8.3)
project(goldorak_navigation)

find_package(catkin REQUIRED
    roslaunch
    move_base
    goldorak_state_publisher
    goldorak_simulation
    goldorak_diffbase
)

catkin_package()
