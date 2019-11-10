project(inf552)

# Compiler and CMake settings
cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 11)

# Libraries
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Include
include_directories(include)

# RANSAC
set(task line_ransac)
set(${PROJECT_NAME}_${task}_src
src/main-${task}.cpp
)
add_executable(${PROJECT_NAME}_${task} ${${PROJECT_NAME}_${task}_src})
target_link_libraries(${PROJECT_NAME}_${task} ${OpenCV_LIBS})

# Panorama
set(task panorama)
set(${PROJECT_NAME}_${task}_src
src/main-${task}.cpp
)
add_executable(${PROJECT_NAME}_${task} ${${PROJECT_NAME}_${task}_src})
target_link_libraries(${PROJECT_NAME}_${task} ${OpenCV_LIBS})
