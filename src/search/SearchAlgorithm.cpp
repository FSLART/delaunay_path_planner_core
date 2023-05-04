//
// Created by carlostojal on 28/04/2023.
//

#include <delaunay_path_planner_core/search/SearchAlgorithm.h>

namespace path_planner::search {

    void SearchAlgorithm::setInitialState(std::shared_ptr<path_planner::State> initial) {
        this->initialState = initial;
    }

    void SearchAlgorithm::setGoalState(std::shared_ptr<path_planner::State> goal) {
        this->goalState = goal;
    }

    size_t SearchAlgorithm::getMaxIterations() const {
        return this->maxIterations;
    }

    void SearchAlgorithm::setMaxIterations(size_t n_iterations) {
        this->maxIterations = n_iterations;
    }

} // path_planner