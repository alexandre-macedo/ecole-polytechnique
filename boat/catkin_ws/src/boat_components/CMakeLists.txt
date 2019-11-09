cmake_minimum_required(VERSION 2.8.3)
project(boat_components)

#set (CMAKE_CXX_STANDARD 14)
set (CMAKE_CXX_FLAGS "--std=gnu++11 ${CMAKE_CXX_FLAGS}")
find_package(catkin REQUIRED COMPONENTS
	roslaunch
	roscpp 
	cv_bridge
	image_transport
	btmx
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
	CATKIN_DEPENDS
	roslaunch
	roscpp 
	btmx
)

roslaunch_add_file_check(launch)

add_executable(motor src/motor/motor.cpp)
target_link_libraries(motor ${catkin_LIBRARIES})

add_executable(servo src/servo/servo.cpp)
target_link_libraries(servo ${catkin_LIBRARIES})

add_executable(imu src/imu/imu.cpp)
target_link_libraries(imu ${catkin_LIBRARIES})

add_executable(camera src/camera/camera.cpp)
target_link_libraries(camera ${catkin_LIBRARIES})


set(${PROJECT_NAME}_NODES
motor
servo
imu
camera
)

install(TARGETS ${${PROJECT_NAME}_NODES}
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
  		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  		PATTERN ".svn" EXCLUDE)
