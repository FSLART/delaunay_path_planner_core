//
// Created by carlostojal on 28/04/2023.
//

#ifndef DELAUNAY_PATH_PLANNER_CORE_HEURISTIC_H
#define DELAUNAY_PATH_PLANNER_CORE_HEURISTIC_H

#include <memory>
#include "path_planner/State.h"

namespace path_planner::search::heuristics {

    class Heuristic {
        public:
            virtual double compute(std::shared_ptr<path_planner::State> state1, std::shared_ptr<path_planner::State> state2,
                                   std::shared_ptr<path_planner::State> goalState);
    };

} // path_planner

#endif //DELAUNAY_PATH_PLANNER_CORE_HEURISTIC_H
