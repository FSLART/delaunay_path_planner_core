//
// Created by carlostojal on 28/04/2023.
//

#ifndef DELAUNAY_PATH_PLANNER_CORE_ASTARHEURISTIC_H
#define DELAUNAY_PATH_PLANNER_CORE_ASTARHEURISTIC_H

#include "Heuristic.h"

namespace path_planner::search::heuristics {

    class AStarHeuristic : Heuristic {
        public:
            double compute(std::shared_ptr<path_planner::State> state1, std::shared_ptr<path_planner::State> state2,
                               std::shared_ptr<path_planner::State> goalState) override;
    };

} // path_planner

#endif //DELAUNAY_PATH_PLANNER_CORE_ASTARHEURISTIC_H
