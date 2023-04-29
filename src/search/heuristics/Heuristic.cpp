//
// Created by carlostojal on 28/04/2023.
//

#include "path_planner/search/heuristics/Heuristic.h"

namespace path_planner::search::heuristics {

    double Heuristic::compute(std::shared_ptr<path_planner::State> state1, std::shared_ptr<path_planner::State> state2,
                              std::shared_ptr<path_planner::State> goalState) {
        return 0;
    }
} // path_planner