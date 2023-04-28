//
// Created by carlostojal on 28/04/2023.
//

#ifndef DELAUNAY_PATH_PLANNER_CORE_ASTAR_H
#define DELAUNAY_PATH_PLANNER_CORE_ASTAR_H

#include <path_planner/SearchAlgorithm.h>
#include <path_planner/AStarHeuristic.h>
#include <queue>
#include <list>

namespace path_planner {

    class AStar : SearchAlgorithm {
        private:
            AStarHeuristic heuristic;

        public:
            std::list<std::shared_ptr<path_planner::State>> search() override;
    };

} // path_planner

#endif //DELAUNAY_PATH_PLANNER_CORE_ASTAR_H
