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

            std::shared_ptr<path_planner::State> currentState;

            visited.insert(this->initialState);
            frontier.push(this->initialState);

            while(!frontier.empty()) {

                // get the next state to explore
                currentState.reset();
                currentState = frontier.front();
                frontier.pop();

                if(this->cmp(currentState, goalState)) {
                    path = SearchAlgorithm::constructPath(parent, currentState);
                    return path;
                }

                /* TODO: solve this issue:
                 * - Consider child 0x55555562a230 is marked as visited.
                 * - It is added to the frontier.
                 * - When it is explored, the "visited" check fails.
                 */

                // explore the first of the frontier
                for(const auto& child : currentState->getNeighbors()) {
                    // this state is not explored
                    if(visited.find(child) == visited.end()) {
                        frontier.push(child);
                        visited.insert(child);
                        parent[child] = currentState;
                        // std::cout << currentState.get() << " has child " << child.get() << "(explored)" << std::endl;
                    }
                }
            }

            return path;
        }
    } // path_planner
} // search