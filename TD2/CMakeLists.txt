cmake_minimum_required(VERSION 2.8.3)
project(TD2)

#include_directories(include)

## Exo 0
set(EXO 0)
set(${PROJECT_NAME}_exo${EXO}_src
src/Complex.cpp
src/testComplex${EXO}.cpp
)
add_executable(${PROJECT_NAME}_exo${EXO} ${${PROJECT_NAME}_exo${EXO}_src})

## Exo 1
set(EXO 1)
set(${PROJECT_NAME}_exo${EXO}_src
src/Complex.cpp
src/testComplex${EXO}.cpp
)
add_executable(${PROJECT_NAME}_exo${EXO} ${${PROJECT_NAME}_exo${EXO}_src})

## Exo 2
set(EXO 2)
set(${PROJECT_NAME}_exo${EXO}_src
src/Complex.cpp
src/testComplex${EXO}.cpp
)
add_executable(${PROJECT_NAME}_exo${EXO} ${${PROJECT_NAME}_exo${EXO}_src})

## Exo 3
set(EXO 3)
set(${PROJECT_NAME}_exo${EXO}_src
src/Complex.cpp
src/testComplex${EXO}.cpp
)
add_executable(${PROJECT_NAME}_exo${EXO} ${${PROJECT_NAME}_exo${EXO}_src})

## Exo 2-1
set(EXO 4)
set(${PROJECT_NAME}_exo${EXO}_src
src/Atom.cpp
src/Rectangle.cpp
src/QuadTree.cpp
src/testQuadTree1.cpp
)

add_executable(${PROJECT_NAME}_exo${EXO} ${${PROJECT_NAME}_exo${EXO}_src})

## Exo 2-2
set(EXO 5)
set(${PROJECT_NAME}_exo${EXO}_src
src/Atom.cpp
src/Rectangle.cpp
src/QuadTree.cpp
src/testQuadTree2.cpp
)

add_executable(${PROJECT_NAME}_exo${EXO} ${${PROJECT_NAME}_exo${EXO}_src})

