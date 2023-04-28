cmake_minimum_required(VERSION 3.25)
project(delaunay_path_planner_core)

set(CMAKE_CXX_STANDARD 17)
include_directories(include)

add_executable(delaunay_path_planner_core main.cpp src/Point.cpp src/State.cpp src/Environment.cpp src/SearchAlgorithm.cpp include/path_planner/SearchAlgorithm.h src/GreedySearch.cpp include/path_planner/GreedySearch.h src/Heuristic.cpp include/path_planner/Heuristic.h)
