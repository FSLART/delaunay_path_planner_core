//
// Created by carlostojal on 28/04/2023.
//

#include <delaunay_path_planner_core/search/heuristics/Heuristic.h>

namespace path_planner::search::heuristics {

    double Heuristic::compute(const std::shared_ptr<path_planner::State>& state1,
                              const std::shared_ptr<path_planner::State>& state2,
                              const std::shared_ptr<path_planner::State>& goalState,
                              double currentGCost) {
        return 0;
    }
} // path_planner