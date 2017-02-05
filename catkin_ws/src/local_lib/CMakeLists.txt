cmake_minimum_required(VERSION 2.8.3)
project(local_lib)


find_package(catkin REQUIRED COMPONENTS
roscpp
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include)

add_library(local_lib src/init_param.cpp)