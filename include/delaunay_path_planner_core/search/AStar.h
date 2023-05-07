//
// Created by carlostojal on 28/04/2023.
//

#ifndef DELAUNAY_PATH_PLANNER_CORE_ASTAR_H
#define DELAUNAY_PATH_PLANNER_CORE_ASTAR_H

#include <delaunay_path_planner_core/search/InformedSearchAlgorithm.h>
#include <delaunay_path_planner_core/search/heuristics/AStarHeuristic.h>
#include <queue>
#include <list>

namespace path_planner::search {

    class AStar : InformedSearchAlgorithm<path_planner::search::heuristics::AStarHeuristic> {

        public:
            path_planner::Path search() override;
    };

} // path_planner

#endif //DELAUNAY_PATH_PLANNER_CORE_ASTAR_H
