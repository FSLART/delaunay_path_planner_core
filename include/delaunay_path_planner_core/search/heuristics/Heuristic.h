//
// Created by carlostojal on 28/04/2023.
//

#ifndef DELAUNAY_PATH_PLANNER_CORE_HEURISTIC_H
#define DELAUNAY_PATH_PLANNER_CORE_HEURISTIC_H

#include <memory>
#include <lart_common/State.h>

namespace path_planner::search::heuristics {

    /*! \brief Abstract heuristic class which computes an heuristic depending on the applied problem and algorithm. */
    class Heuristic {
        public:
            virtual double compute(const std::shared_ptr<lart_common::State>& state1,
                                   const std::shared_ptr<lart_common::State>& state2,
                                   const std::shared_ptr<lart_common::State>& goalState,
                                   double currentGCost) = 0;
    };

} // path_planner

#endif //DELAUNAY_PATH_PLANNER_CORE_HEURISTIC_H
