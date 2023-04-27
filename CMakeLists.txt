cmake_minimum_required(VERSION 3.25)
project(delaunay_path_planner_core)

set(CMAKE_CXX_STANDARD 17)

add_executable(delaunay_path_planner_core main.cpp src/Point.cpp include/Point.h)
