# Check if this file has already been included.
if(NOT TARGET pcl_aggregator_core)
    # Define the target.
    add_library(delaunay_path_planner_core SHARED IMPORTED)
    set_target_properties(delaunay_path_planner_core PROPERTIES IMPORTED_LOCATION /usr/lib/delaunay_path_planner_core/libdelaunay_path_planner_core.so)

    # Set the dependencies.
    find_package(CGAL REQUIRED)

    target_include_directories(delaunay_path_planner_core INTERFACE /usr/include/delaunay_path_planner_core)
    target_link_libraries(delaunay_path_planner_core INTERFACE ${CGAL_LIBRARIES})
endif()