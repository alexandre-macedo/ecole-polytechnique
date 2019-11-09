cmake_minimum_required(VERSION 2.8.3)
project(boat_bringup)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roslaunch
)

catkin_package(
   CATKIN_DEPENDS roslaunch
)

include_directories(include ${catkin_INCLUDE_DIRS})
roslaunch_add_file_check(launch)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    PATTERN ".svn" EXCLUDE)