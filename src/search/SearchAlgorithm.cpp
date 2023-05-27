//
// Created by carlostojal on 28/04/2023.
//

#include <delaunay_path_planner_core/search/SearchAlgorithm.h>

namespace path_planner::search {

    void SearchAlgorithm::setInitialState(const std::shared_ptr<path_planner::State>& initial) {
        this->initialState = initial;
    }

    void SearchAlgorithm::setGoalState(const std::shared_ptr<path_planner::State>& goal) {
        this->goalState = goal;
    }

    size_t SearchAlgorithm::getMaxIterations() const {
        return this->maxIterations;
    }

    void SearchAlgorithm::setMaxIterations(size_t n_iterations) {
        this->maxIterations = n_iterations;
    }

    void SearchAlgorithm::setComparator(std::function<bool(const std::shared_ptr<path_planner::State> &,
                                                           const std::shared_ptr<path_planner::State> &)> cmp) {
        this->cmp = std::move(cmp);
    }

    path_planner::Path SearchAlgorithm::constructPath(const std::unordered_map<std::shared_ptr<path_planner::State>,
            std::shared_ptr<path_planner::State>>& parent,
            std::shared_ptr<path_planner::State>& currentState) {

        path_planner::Path path;

        std::shared_ptr<path_planner::State> iter = currentState;

        while (iter != nullptr) {
            // std::cout << iter.get() << "'s parent is " << parent.at(iter) << std::endl;
            path.prependState(iter);
            if(!parent.contains(iter))
                iter = nullptr;
            else
                iter = parent.at(iter);
        }

        return path;
    }

} // path_planner