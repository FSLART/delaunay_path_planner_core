//
// Created by carlostojal on 28/04/2023.
//

#include <path_planner/AStar.h>

#define MAX_SEARCH_ITERATIONS 200

namespace path_planner {

    std::list<std::shared_ptr<path_planner::State>> AStar::search() {

        if(this->initialState == nullptr)
            throw std::runtime_error("Initial state not set!");

        if(this->goalState == nullptr)
            throw std::runtime_error("Goal state not set!");

        // initialize the frontier
        std::list<std::shared_ptr<path_planner::State>> frontier;
        frontier.push_back(this->initialState);

        // minimum cost of all the frontier
        std::shared_ptr<path_planner::State> currMinCostNode = nullptr;
        std::shared_ptr<path_planner::State> stateToExplore = this->initialState;
        double minFrontierCost = std::numeric_limits<double>::infinity();

        std::list<std::shared_ptr<path_planner::State>> actionList;

        // the node currently being explored
        actionList.push_back(this->initialState);

        while(currMinCostNode != this->goalState) {

            // find the frontier minimum cost
            for (auto & frontierIter : frontier) {
                double h = this->heuristic.compute(currMinCostNode, frontierIter, this->goalState);
                if(h < minFrontierCost) {
                    minFrontierCost = h;
                    currMinCostNode = frontierIter;
                }
            }

            // check if it is the first exploration
            if(stateToExplore != this->initialState) {
                // the new minimum is not a neighbor of the last explored node, meaning the search changed branch
                if (stateToExplore->hasNeighbor(currMinCostNode)) {

                    // remove the last explored node from the action list
                    actionList.remove(stateToExplore);
                }
            }

            // the current minimum is the new node to explore
            stateToExplore = currMinCostNode;
            actionList.push_back(stateToExplore);

            // remove the node to be explored from the frontier, because it will be replaced by its neighbors
            frontier.remove(stateToExplore);

            // explore the node
            for(auto & neighborIter : stateToExplore->getNeighbors())
                frontier.push_back(neighborIter);

        }

        return actionList;
    }
} // path_planner