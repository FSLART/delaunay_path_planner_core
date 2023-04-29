//
// Created by carlostojal on 28/04/2023.
//

#ifndef DELAUNAY_PATH_PLANNER_CORE_ASTAR_H
#define DELAUNAY_PATH_PLANNER_CORE_ASTAR_H

#include "InformedSearchAlgorithm.h"
#include "path_planner/search/heuristics/AStarHeuristic.h"
#include <queue>
#include <list>

namespace path_planner::search {

    class AStar : InformedSearchAlgorithm<path_planner::search::heuristics::AStarHeuristic> {

        public:
            std::list<std::shared_ptr<path_planner::State>> search() override;
    };

} // path_planner

#endif //DELAUNAY_PATH_PLANNER_CORE_ASTAR_H
