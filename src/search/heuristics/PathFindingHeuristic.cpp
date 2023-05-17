//
// Created by carlostojal on 06-05-2023.
//

#include <delaunay_path_planner_core/search/heuristics/PathFindingHeuristic.h>

namespace path_planner {
    namespace search {
        namespace heuristics {

            void findClosestConeRoutine(PathFindingHeuristic *instance, cone_color_t c,
                                        std::shared_ptr<path_planner::State> initialState,
                                        std::shared_ptr<path_planner::State>* found) {
                *found = std::move(instance->coneFindingSearch(c, std::move(initialState)));
            }

            double PathFindingHeuristic::compute(const std::shared_ptr<path_planner::State>& state1,
                                                 const std::shared_ptr<path_planner::State>& state2,
                                                 const std::shared_ptr<path_planner::State>& goalState,
                                                 double currentGCost) {

                // find the closest blue cone and compute distance
                std::shared_ptr<path_planner::State> closestBlue;
                std::thread blueSearchThread(findClosestConeRoutine, this, BLUE, state2, &closestBlue);

                // find the closest yellow cone and compute distance
                std::shared_ptr<path_planner::State> closestYellow;
                std::thread yellowSearchThread(findClosestConeRoutine, this, YELLOW, state2, &closestYellow);

                // wait the threads
                blueSearchThread.join();
                yellowSearchThread.join();

                if(closestYellow == nullptr)
                    throw std::runtime_error("No yellow cone exists in the horizon!");
                if(closestBlue == nullptr)
                    throw std::runtime_error("No blue cone exists in the horizon!");

                /* return absolute difference of distances as heuristic. if the distances are too big, it means
                 * that node is deviated from the center line */
                double blueDistance = state2->getPosition().distanceTo(closestBlue->getPosition());
                double yellowDistance = state2->getPosition().distanceTo(closestYellow->getPosition());

                return abs(blueDistance - yellowDistance);
            }

            std::shared_ptr<path_planner::State> PathFindingHeuristic::coneFindingSearch(cone_color_t c,
                                                                   std::shared_ptr<path_planner::State> initialState) {

                if(initialState == nullptr)
                    throw std::runtime_error("Tried to perform a search without a set initial state!");

                // store the frontier to explore
                std::queue<std::shared_ptr<path_planner::State>> frontier;
                frontier.push(initialState);

                std::set<std::shared_ptr<path_planner::State>> visited;
                visited.insert(initialState);

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
                    if(State::occupancyTypeToConeColor(s->getOccupancy()) == c) {
                        return s;
                    }

                    for(const auto& iter : s->getNeighbors()) {
                        if(visited.count(iter) == 0) {
                            visited.insert(iter);
                            frontier.push(iter);
                        }
                    }
                }

                s.reset();
                initialState.reset();

                return nullptr;
            }

        } // path_planner
    } // search
} // heuristics