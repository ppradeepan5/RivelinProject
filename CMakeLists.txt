cmake_minimum_required(VERSION 3.16)
project(KinematicsProject)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

Include_directories(
    robot
    planning
)

file (GLOB_RECURSE SOURCES
    src/*.cpp
    robot/*.cpp
    planning/*.cpp
)

add_executable(KinematicsProject ${SOURCES})