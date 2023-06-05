//
// Created by carlostojal on 28/04/2023.
//

#include <delaunay_path_planner_core/search/heuristics/AStarHeuristic.h>

namespace path_planner::search::heuristics {

    double AStarHeuristic::compute(const std::shared_ptr<lart_common::State>& state1,
                                   const std::shared_ptr<lart_common::State>& state2,
                                   const std::shared_ptr<lart_common::State>& goalState, double currentGCost = 0) {

        if(!state1->getNeighbors().count(state2))
            throw std::runtime_error("State transition not possible!");

        // f(n) = g(n) + h(n)
        double fn, gn;
        fn = state1->getPosition().distanceTo(state2->getPosition());
        gn = state1->getPosition().distanceTo(goalState->getPosition());

        return fn + gn;
    }
} // path_planner