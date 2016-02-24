cmake_minimum_required(VERSION 2.8.3)
project(goldorak_simulation)

find_package(catkin REQUIRED COMPONENTS
    roslaunch
)
catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
)

roslaunch_add_file_check(launch)

install(DIRECTORY config images launch rviz urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
