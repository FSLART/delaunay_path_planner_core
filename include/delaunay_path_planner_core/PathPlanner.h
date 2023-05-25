//
// Created by carlostojal on 05-05-2023.
//

#ifndef DELAUNAY_PATH_PLANNER_CORE_PATHPLANNER_H
#define DELAUNAY_PATH_PLANNER_CORE_PATHPLANNER_H

#include <delaunay_path_planner_core/Path.h>
#include <delaunay_path_planner_core/Environment.h>
#include <delaunay_path_planner_core/search/heuristics/PathFindingHeuristic.h>
#include <delaunay_path_planner_core/search/AStar.h>
#include <list>

namespace path_planner {

    /*! @brief This class offers path planning functionalities.
     */
    class PathPlanner {

        /*! @brief Search a list of potential paths on the environment.
         *
         * @param e The environment to search.
         *
         * @return The list of potential paths on the given environment.
         * */
        static std::list<path_planner::Path> search(const Environment& e);

    };

} // path_planner

#endif //DELAUNAY_PATH_PLANNER_CORE_PATHPLANNER_H
