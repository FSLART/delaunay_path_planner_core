//
// Created by carlostojal on 28/04/2023.
//

#include <delaunay_path_planner_core/search/GreedySearch.h>

#define MAX_SEARCH_ITERATIONS 200

namespace path_planner::search {

    std::list<std::shared_ptr<path_planner::State>> GreedySearch::search() {
        if (this->initialState == nullptr)
            throw std::runtime_error("Initial state not set on search!");

        if (this->goalState == nullptr)
            throw std::runtime_error("Goal state not set on search!");

        std::list<std::shared_ptr<path_planner::State>> actionList;
        actionList.push_back(this->initialState);

        double minCost = std::numeric_limits<double>::infinity();
        std::shared_ptr<path_planner::State> minCostState = nullptr;

        size_t iterationCount = 0;

        std::shared_ptr<path_planner::State> currState = this->initialState;
        while (currState != goalState && iterationCount < MAX_SEARCH_ITERATIONS) {

            for (auto neighborIter = currState->getNeighbors().begin();
                 neighborIter != currState->getNeighbors().end(); neighborIter++) {
                double computedCost = this->heuristic.compute(currState, *neighborIter, this->goalState);
                if (computedCost < minCost) {
                    // reset the min cost state
                    minCost = computedCost;
                    minCostState = *neighborIter;
                }
            }

            // add the best action to the list
            actionList.push_back(minCostState);
            currState = minCostState;

            iterationCount++;
        }

        return actionList;
    }
} // path_planner