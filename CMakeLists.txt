cmake_minimum_required(VERSION 3.25)
project(delaunay_path_planner_core)

set(CMAKE_CXX_STANDARD 17)

find_package(CGAL REQUIRED)

include_directories(include ${CGAL_INCLUDE_DIRS})

add_executable(delaunay_path_planner_core main.cpp src/Point.cpp src/State.cpp src/Environment.cpp src/SearchAlgorithm.cpp include/path_planner/SearchAlgorithm.h src/GreedySearch.cpp include/path_planner/GreedySearch.h src/GreedyHeuristic.cpp include/path_planner/GreedyHeuristic.h src/Heuristic.cpp include/path_planner/Heuristic.h src/AStar.cpp include/path_planner/AStar.h src/AStarHeuristic.cpp include/path_planner/AStarHeuristic.h)
target_link_libraries(delaunay_path_planner_core ${CGAL_LIBS})
