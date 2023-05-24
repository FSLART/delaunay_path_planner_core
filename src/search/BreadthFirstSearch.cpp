//
// Created by carlostojal on 24-05-2023.
//

#include "delaunay_path_planner_core/search/BreadthFirstSearch.h"

namespace path_planner {
    namespace search {
        path_planner::Path BreadthFirstSearch::search() {

            path_planner::Path path;

            std::queue<std::shared_ptr<path_planner::State>> frontier;
            // to track visited nodes
            std::unordered_set<std::shared_ptr<path_planner::State>> visited;
            // to track parents of the states
            std::unordered_map<std::shared_ptr<path_planner::State>,std::shared_ptr<path_planner::State>> parent;

            frontier.push(this->initialState);
            visited.insert(this->initialState);

            while(!frontier.empty()) {

                // get the next state to explore
                std::shared_ptr<path_planner::State> currentState = frontier.front();
                frontier.pop();

                if(this->cmp(currentState, goalState)) {
                    std::shared_ptr<path_planner::State> pathIter = currentState;
                    // add the states to the path
                    while(pathIter != nullptr) {
                        path.prependState(pathIter);
                        pathIter = parent.at(pathIter);
                    }
                }

                // explore the first of the frontier
                for(const auto& child : frontier.front()->getNeighbors()) {
                    if(visited.find(child) == visited.end()) {
                        frontier.push(child);
                        visited.insert(child);
                        parent[child] = currentState;
                    }
                }

                // remove the first of the frontier
                frontier.pop();
            }
        }
    } // path_planner
} // search