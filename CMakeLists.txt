cmake_minimum_required(VERSION 3.25)
project(delaunay_path_planner_core)

set(CMAKE_CXX_STANDARD 17)

find_package(CGAL REQUIRED)

include_directories(include ${CGAL_INCLUDE_DIRS})

add_executable(delaunay_path_planner_core main.cpp src/Point.cpp src/State.cpp src/Environment.cpp src/search/SearchAlgorithm.cpp include/path_planner/search/SearchAlgorithm.h src/search/GreedySearch.cpp include/path_planner/search/GreedySearch.h src/search/heuristics/GreedyHeuristic.cpp include/path_planner/search/heuristics/GreedyHeuristic.h src/search/heuristics/Heuristic.cpp include/path_planner/search/heuristics/Heuristic.h src/search/AStar.cpp include/path_planner/search/AStar.h src/search/heuristics/AStarHeuristic.cpp include/path_planner/search/heuristics/AStarHeuristic.h src/search/DepthFirstSearch.cpp include/path_planner/search/DepthFirstSearch.h src/search/InformedSearchAlgorithm.cpp include/path_planner/search/InformedSearchAlgorithm.h)
target_link_libraries(delaunay_path_planner_core ${CGAL_LIBRARIES})
