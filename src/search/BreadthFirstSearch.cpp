//
// Created by carlostojal on 06-05-2023.
//

#include "delaunay_path_planner_core/search/BreadthFirstSearch.h"

namespace path_planner {
    namespace search {
        path_planner::Path BreadthFirstSearch::search() {

            if(this->initialState == nullptr)
                throw new std::runtime_error("Tried to perform a search without a set initial state!");

            if(this->goalState == nullptr)
                throw new std::runtime_error("Tried to perform a search without a set goal state!");

            // start the path
            Path path;
            path.addState(this->initialState, 0);

            // store the frontier to explore
            std::queue<std::shared_ptr<path_planner::State>> frontier;
            frontier.push(this->initialState);

            std::set<std::shared_ptr<path_planner::State>> visited;
            visited.insert(this->initialState);

            // iterator
            std::shared_ptr<path_planner::State> s;

            while(!frontier.empty()) {
                // free the old value
                s.reset();
                // access the next state of the queue
                s = frontier.front();
                // remove that state from the queue
                frontier.pop();

                // check if it is the goal
                if(s == this->goalState) {
                    path.addState(s, 1);
                    return path;
                }
                for(const auto& iter : s->getNeighbors()) {
                    if(visited.count(iter) == 0) {
                        visited.insert(iter);
                        frontier.push(iter);
                    }
                }
            }

            s.reset();

            return path;
        }
    } // path_planner
} // search