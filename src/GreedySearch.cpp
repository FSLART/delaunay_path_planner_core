//
// Created by 2211006 on 28/04/2023.
//

#include <path_planner/GreedySearch.h>

namespace path_planner {

    std::list<std::shared_ptr<path_planner::State>> GreedySearch::search() {
        if(this->initialState == nullptr)
            throw std::runtime_error("Initial state not set on search!");

        if(this->goalState == nullptr)
            throw std::runtime_error("Goal state not set on search!");
    }
} // path_planner