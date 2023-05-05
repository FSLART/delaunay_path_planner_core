//
// Created by carlostojal on 28/04/2023.
//

#ifndef DELAUNAY_PATH_PLANNER_CORE_GREEDYHEURISTIC_H
#define DELAUNAY_PATH_PLANNER_CORE_GREEDYHEURISTIC_H

#include <delaunay_path_planner_core/State.h>
#include <limits>
#include <delaunay_path_planner_core/search/heuristics/Heuristic.h>

namespace path_planner::search::heuristics {

    class GreedyHeuristic : Heuristic {

        public:
            double compute(std::shared_ptr<path_planner::State> state1, std::shared_ptr<path_planner::State> state2,
                           std::shared_ptr<path_planner::State> goalState) override;
    };

} // path_planner

#endif //DELAUNAY_PATH_PLANNER_CORE_GREEDYHEURISTIC_H
