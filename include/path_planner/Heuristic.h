//
// Created by carlostojal on 28/04/2023.
//

#ifndef DELAUNAY_PATH_PLANNER_CORE_HEURISTIC_H
#define DELAUNAY_PATH_PLANNER_CORE_HEURISTIC_H

#include <path_planner/State.h>
#include <limits>

namespace path_planner {

    class Heuristic {

        public:
            static double compute(std::shared_ptr<path_planner::State> state1, std::shared_ptr<path_planner::State> state2);
    };

} // path_planner

#endif //DELAUNAY_PATH_PLANNER_CORE_HEURISTIC_H
