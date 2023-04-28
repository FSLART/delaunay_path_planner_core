//
// Created by carlostojal on 28/04/2023.
//

#include "path_planner/AStarHeuristic.h"

namespace path_planner {
    double
    AStarHeuristic::compute(std::shared_ptr<path_planner::State> state1, std::shared_ptr<path_planner::State> state2,
                            std::shared_ptr<path_planner::State> goalState) {

        if(!state1->getNeighbors().count(state2))
            throw std::runtime_error("State transition not possible!");

        // h(n) = f(n) + g(n)
        double fn, gn;
        fn = state1->getPosition().distanceTo(state2->getPosition());
        gn = state1->getPosition().distanceTo(goalState->getPosition());

        return fn + gn;
    }
} // path_planner