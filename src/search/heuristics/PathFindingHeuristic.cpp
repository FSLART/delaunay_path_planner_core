//
// Created by carlostojal on 06-05-2023.
//

#include <delaunay_path_planner_core/search/heuristics/PathFindingHeuristic.h>

namespace path_planner {
    namespace search {
        namespace heuristics {

            void PathFindingHeuristic::findClosestConeRoutine(lart_common::occupancy_type_t t,
                                                                     const std::shared_ptr<lart_common::State>& initialState,
                                                                     std::shared_ptr<lart_common::State>& found) {

                std::shared_ptr<lart_common::State> dummyState = std::make_shared<lart_common::State>();
                dummyState->setOccupancy(t);

                // lambda function to compare two states in this search problem
                auto compareStatesByConeColor = [](const std::shared_ptr<lart_common::State>& s1,
                                                   const std::shared_ptr<lart_common::State>& s2) {
                    return s1->getOccupancy() == s2->getOccupancy();
                };

                // find the closest cone of each color for this state
                // use breadth first search
                BreadthFirstSearch bfs = BreadthFirstSearch();
                bfs.setComparator(compareStatesByConeColor);
                bfs.setInitialState(initialState);

                // find the closest yellow
                bfs.setGoalState(dummyState);
                lart_common::Path path = bfs.search();

                found = path.getFullPath().back();
            }

            double PathFindingHeuristic::compute(const std::shared_ptr<lart_common::State>& state1,
                                                 const std::shared_ptr<lart_common::State>& state2,
                                                 const std::shared_ptr<lart_common::State>& goalState,
                                                 double currentGCost) {


                std::vector<std::thread> threads = std::vector<std::thread>();

                std::shared_ptr<lart_common::State> closestYellowCone = nullptr;

                if(state2->getOccupancy() == lart_common::YELLOW_CONE_OCCUPANCY) {
                    closestYellowCone = state2;
                } else { // start a thread to search the closest yellow
                    threads.emplace_back(findClosestConeRoutine, lart_common::YELLOW_CONE_OCCUPANCY, std::ref(state2),
                                         std::ref(closestYellowCone));
                }


                std::shared_ptr<lart_common::State> closestBlueCone = nullptr;

                if(state2->getOccupancy() == lart_common::BLUE_CONE_OCCUPANCY) {
                    closestBlueCone = state2;
                } else { // start a thread to search the closest blue
                    threads.emplace_back(findClosestConeRoutine, lart_common::BLUE_CONE_OCCUPANCY, std::ref(state2),
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