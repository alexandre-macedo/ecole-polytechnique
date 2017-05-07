cmake_minimum_required(VERSION 2.8.3)
project(ray_tracer)

set(${PROJECT_NAME}_src
src/main.cpp
src/rt_vector.cpp
src/rt_ray.cpp
src/rt_sphere.cpp
src/rt_scene.cpp
src/rt_camera.cpp
)

add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_src})