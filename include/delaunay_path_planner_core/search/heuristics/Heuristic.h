//
// Created by carlostojal on 28/04/2023.
//

#ifndef DELAUNAY_PATH_PLANNER_CORE_HEURISTIC_H
#define DELAUNAY_PATH_PLANNER_CORE_HEURISTIC_H

#include <memory>
#include <delaunay_path_planner_core/State.h>

namespace path_planner::search::heuristics {

    class Heuristic {
        public:
            virtual double compute(const std::shared_ptr<path_planner::State>& state1,
                                   const std::shared_ptr<path_planner::State>& state2,
                                   const std::shared_ptr<path_planner::State>& goalState,
                                   double currentGCost = 0) = 0;
    };

} // path_planner

#endif //DELAUNAY_PATH_PLANNER_CORE_HEURISTIC_H
