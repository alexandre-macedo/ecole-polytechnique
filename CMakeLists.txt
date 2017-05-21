cmake_minimum_required(VERSION 3.1.0 FATAL_ERROR)
project(ray_tracer)

set(${PROJECT_NAME}_src
src/main.cpp
src/rt_vector.cpp
src/rt_ray.cpp
src/rt_sphere.cpp
src/rt_scene.cpp
src/rt_light.cpp
src/rt_camera.cpp
src/rt_tools.cpp
src/rt_color.cpp
src/rt_image.cpp
)

add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_src})

target_compile_features(${PROJECT_NAME} PRIVATE cxx_range_for)