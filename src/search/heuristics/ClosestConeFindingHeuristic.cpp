//
// Created by carlostojal on 17-05-2023.
//

#include "delaunay_path_planner_core/search/heuristics/ClosestConeFindingHeuristic.h"

namespace path_planner {
    namespace search {
        namespace heuristics {
            double ClosestConeFindingHeuristic::compute(const std::shared_ptr<lart_common::State>& state1,
                                                        const std::shared_ptr<lart_common::State>& state2,
                                                        const std::shared_ptr<lart_common::State>& goalState,
                                                        double currentGCost) {
                // f(n) = g(n)
                return currentGCost + state1->getPosition().distanceTo(state2->getPosition());

            }
        } // path_planner
    } // search
} // heuristics