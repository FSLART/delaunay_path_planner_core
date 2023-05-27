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


                // start a thread to search the closest yellow
                std::shared_ptr<path_planner::State> closestYellowCone = nullptr;
                std::thread yellowFindingThread([state2, &closestYellowCone] { return findClosestConeRoutine(YELLOW_CONE_OCCUPANCY, state2, closestYellowCone); });

                // start a thread to search the closest blue
                std::shared_ptr<path_planner::State> closestBlueCone = nullptr;
                std::thread blueFindingThread([state2, &closestBlueCone] { return findClosestConeRoutine(BLUE_CONE_OCCUPANCY, state2, closestBlueCone); });

                // wait for both threads
                yellowFindingThread.join();
                blueFindingThread.join();

                // compute the distances
                double distanceToYellow = closestYellowCone->getPosition().distanceTo(state1->getPosition());
                double distanceToBlue = closestBlueCone->getPosition().distanceTo(state1->getPosition());

                return abs(distanceToBlue - distanceToYellow);
            }

        } // path_planner
    } // search
} // heuristics