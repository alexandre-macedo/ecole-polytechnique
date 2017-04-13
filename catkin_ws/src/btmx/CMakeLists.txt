cmake_minimum_required(VERSION 2.8.3)
project(btmx)

find_package(catkin REQUIRED COMPONENTS
roscpp
message_generation
std_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  TeleCommands.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
        CATKIN_DEPENDS roscpp message_runtime std_msgs
        LIBRARIES ${PROJECT_NAME}
        INCLUDE_DIRS include
)

set(${PROJECT_NAME}_SRCS
    src/init_param.cpp
)

set(${PROJECT_NAME}_INCLUDES
    include/btmx/init_param.h
)

add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})


# Install executables
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install headers
install(FILES ${${PROJECT_NAME}_INCLUDES}
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME})