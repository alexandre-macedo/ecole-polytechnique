cmake_minimum_required(VERSION 2.8.3)
project(boat_components)

#set (CMAKE_CXX_STANDARD 11)
set (CMAKE_CXX_FLAGS "--std=gnu++11 ${CMAKE_CXX_FLAGS}")
find_package(catkin REQUIRED COMPONENTS
	roslaunch
	roscpp 
	serial
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${CMAKE_SOURCE_DIR}/btmx/include/)

roslaunch_add_file_check(launch)

add_executable(xbee_send src/xbee/xbee_send.cpp)
target_link_libraries(xbee_send ${catkin_LIBRARIES} btmx)

add_executable(xbee_receive src/xbee/xbee_receive.cpp)
target_link_libraries(xbee_receive ${catkin_LIBRARIES} btmx)

add_executable(motor src/motor/motor.cpp)
target_link_libraries(motor ${catkin_LIBRARIES})

add_executable(servo src/servo/servo.cpp)
target_link_libraries(servo ${catkin_LIBRARIES})

add_executable(gps src/gps/gps.cpp)
target_link_libraries(gps ${catkin_LIBRARIES})

add_executable(imu src/imu/imu.cpp)
target_link_libraries(imu ${catkin_LIBRARIES})

add_executable(camera src/camera/camera.cpp)
target_link_libraries(camera ${catkin_LIBRARIES})
