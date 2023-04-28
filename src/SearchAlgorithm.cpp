//
// Created by carlostojal on 28/04/2023.
//

#include "path_planner/SearchAlgorithm.h"

namespace path_planner {

    void SearchAlgorithm::setInitialState(std::shared_ptr<path_planner::State> initial) {
        this->initialState = initial;
    }

    void SearchAlgorithm::setGoalState(std::shared_ptr<path_planner::State> goal) {
        this->goalState = goal;
    }
} // path_planner