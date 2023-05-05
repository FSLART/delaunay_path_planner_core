//
// Created by carlostojal on 05-05-2023.
//

#ifndef DELAUNAY_PATH_PLANNER_CORE_PATHPLANNER_H
#define DELAUNAY_PATH_PLANNER_CORE_PATHPLANNER_H

#include <delaunay_path_planner_core/Path.h>
#include <list>

namespace path_planner {

    class PathPlanner {

        static std::list<path_planner::Path> search();

    };

} // path_planner

#endif //DELAUNAY_PATH_PLANNER_CORE_PATHPLANNER_H
