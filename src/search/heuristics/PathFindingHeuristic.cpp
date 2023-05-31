//
// Created by carlostojal on 06-05-2023.
//

#include <delaunay_path_planner_core/search/heuristics/PathFindingHeuristic.h>

namespace path_planner {
    namespace search {
        namespace heuristics {

            void PathFindingHeuristic::findClosestConeRoutine(occupancy_type_t t,
                                                                     const std::shared_ptr<path_planner::State>& initialState,
                                                                     std::shared_ptr<path_planner::State>& found) {

                std::shared_ptr<path_planner::State> dummyState = std::make_shared<path_planner::State>();
                dummyState->setOccupancy(t);

                // lambda function to compare two states in this search problem
                auto compareStatesByConeColor = [](const std::shared_ptr<path_planner::State>& s1,
                                                   const std::shared_ptr<path_planner::State>& s2) {
                    return s1->getOccupancy() == s2->getOccupancy();
                };

                // find the closest cone of each color for this state
                // use breadth first search
                BreadthFirstSearch bfs = BreadthFirstSearch();
                bfs.setComparator(compareStatesByConeColor);
                bfs.setInitialState(initialState);

                // find the closest yellow
                bfs.setGoalState(dummyState);
                path_planner::Path path = bfs.search();

                found = path.getFullPath().back();
            }

            double PathFindingHeuristic::compute(const std::shared_ptr<path_planner::State>& state1,
                                                 const std::shared_ptr<path_planner::State>& state2,
                                                 const std::shared_ptr<path_planner::State>& goalState,
                                                 double currentGCost) {


                std::vector<std::thread> threads = std::vector<std::thread>();

                std::shared_ptr<path_planner::State> closestYellowCone = nullptr;

                if(state2->getOccupancy() == YELLOW_CONE_OCCUPANCY) {
                    closestYellowCone = state2;
                } else { // start a thread to search the closest yellow
                    threads.emplace_back(findClosestConeRoutine, YELLOW_CONE_OCCUPANCY, std::ref(state2),
                                         std::ref(closestYellowCone));
                }


                std::shared_ptr<path_planner::State> closestBlueCone = nullptr;

                if(state2->getOccupancy() == BLUE_CONE_OCCUPANCY) {
                    closestBlueCone = state2;
                } else { // start a thread to search the closest blue
                    threads.emplace_back(findClosestConeRoutine, BLUE_CONE_OCCUPANCY, std::ref(state2),
                                         std::ref(closestBlueCone));
                }

                for(auto & t : threads) {
                    if(t.joinable())
                        t.join();
                }

                // compute the distances
                double distanceToYellow = closestYellowCone->getPosition().distanceTo(state2->getPosition());
                double distanceToBlue = closestBlueCone->getPosition().distanceTo(state2->getPosition());

                return abs(distanceToBlue - distanceToYellow);
            }

        } // path_planner
    } // search
} // heuristics