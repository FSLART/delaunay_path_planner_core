//
// Created by carlostojal on 28/04/2023.
//

#include "path_planner/GreedyHeuristic.h"

namespace path_planner {
    double GreedyHeuristic::compute(std::shared_ptr<path_planner::State> state1, std::shared_ptr<path_planner::State> state2,
                                    std::shared_ptr<path_planner::State> goalState) {
        if(state1->getNeighbors().count(state2) == 0)
            throw std::runtime_error("Impossible to transition to state!");

        if(state2->getOccupancy() != FREE_SPACE)
            return std::numeric_limits<double>::infinity();

        return state1->getPosition().distanceTo(state2->getPosition());
    }
} // path_planner