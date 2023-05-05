cmake_minimum_required(VERSION 3.25)
project(delaunay_path_planner_core)

set(CMAKE_CXX_STANDARD 17)

find_package(CGAL REQUIRED)

include_directories(include ${CGAL_INCLUDE_DIRS})

add_library(delaunay_path_planner_core SHARED src/Point.cpp src/State.cpp src/Environment.cpp src/search/SearchAlgorithm.cpp src/search/GreedySearch.cpp src/search/heuristics/GreedyHeuristic.cpp src/search/heuristics/Heuristic.cpp src/search/AStar.cpp src/search/heuristics/AStarHeuristic.cpp src/search/InformedSearchAlgorithm.cpp src/Cone.cpp)
target_link_libraries(delaunay_path_planner_core ${CGAL_LIBRARIES})

install(TARGETS delaunay_path_planner_core LIBRARY DESTINATION /usr/lib/delaunay_path_planner_core)
install(DIRECTORY include/delaunay_path_planner_core DESTINATION /usr/include)
install(FILES delaunay_path_planner_coreConfig.cmake DESTINATION /usr/lib/delaunay_path_planner_core)