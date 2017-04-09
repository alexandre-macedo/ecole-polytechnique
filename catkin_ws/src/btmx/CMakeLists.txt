cmake_minimum_required(VERSION 2.8.3)
project(btmx)

find_package(catkin REQUIRED COMPONENTS
roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
        LIBRARIES ${PROJECT_NAME}
        INCLUDE_DIRS include
)

set(btmx_SRCS
    src/init_param.cpp
    include/btmx/init_param.h
)

add_library(${PROJECT_NAME} ${btmx_SRCS})


# Install executables
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install headers
install(FILES include/btmx/init_param.h
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/btmx)