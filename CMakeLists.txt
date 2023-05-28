cmake_minimum_required(VERSION 3.22)
project(delaunay_path_planner_core)

set(CMAKE_CXX_STANDARD 20)

find_package(CGAL REQUIRED)

include_directories(include ${CGAL_INCLUDE_DIRS} ${gtest_INCLUDE_DIRS})

add_subdirectory(lib/googletest)

add_library(delaunay_path_planner_core SHARED src/Point.cpp src/State.cpp src/Environment.cpp
        src/search/SearchAlgorithm.cpp src/search/heuristics/Heuristic.cpp src/search/AStar.cpp
        src/search/heuristics/AStarHeuristic.cpp src/search/InformedSearchAlgorithm.cpp src/Cone.cpp
        src/PathPlanner.cpp src/Path.cpp src/search/heuristics/PathFindingHeuristic.cpp test/EnvironmentTests.cpp
        src/search/heuristics/ClosestConeFindingHeuristic.cpp src/search/BreadthFirstSearch.cpp )
target_link_libraries(delaunay_path_planner_core ${CGAL_LIBRARIES})

add_executable(test test/Misc.cpp test/ConeSearchTests.cpp test/EnvironmentTests.cpp test/GeometricTests.cpp test/PathFindingTests.cpp)
target_link_libraries(test delaunay_path_planner_core gtest gtest_main)

install(TARGETS delaunay_path_planner_core LIBRARY DESTINATION /usr/lib/delaunay_path_planner_core)
install(DIRECTORY include/delaunay_path_planner_core DESTINATION /usr/include)
install(FILES delaunay_path_planner_coreConfig.cmake DESTINATION /usr/lib/delaunay_path_planner_core)
