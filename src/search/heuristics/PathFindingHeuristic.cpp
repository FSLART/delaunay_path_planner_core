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



            }

            bool compareStatesByConeColor(const std::shared_ptr<path_planner::State>& s1,
                                          const std::shared_ptr<path_planner::State>& s2) {
                return s1->getOccupancy() == s2->getOccupancy();
            }

            double PathFindingHeuristic::compute(const std::shared_ptr<path_planner::State>& state1,
                                                 const std::shared_ptr<path_planner::State>& state2,
                                                 const std::shared_ptr<path_planner::State>& goalState,
                                                 double currentGCost) {

                // define a dummy yellow cone. the occupancy type is all that matters
                std::shared_ptr<path_planner::State> dummyYellow = std::make_shared<path_planner::State>();
                dummyYellow->setOccupancy(YELLOW_CONE_OCCUPANCY);

                // define a dummy blue cone
                std::shared_ptr<path_planner::State> dummyBlue = std::make_shared<path_planner::State>();
                dummyBlue->setOccupancy(BLUE_CONE_OCCUPANCY);

                // find the closest cone of each color for this state
                // use breadth first search
                BreadthFirstSearch bfs = BreadthFirstSearch();
                bfs.setComparator(compareStatesByConeColor);
                bfs.setInitialState(state2);

                // find the closest yellow
                bfs.setGoalState(dummyYellow);
                path_planner::Path yellowPath = bfs.search();

                // find the closest blue
                bfs.setGoalState(dummyBlue);
                path_planner::Path bluePath = bfs.search();

                // the cones are the last states of the path
                std::shared_ptr<path_planner::State> yellowCone = yellowPath.getFullPath().back();
                std::shared_ptr<path_planner::State> blueCone = bluePath.getFullPath().back();

                // compute the distances
                double distanceToYellow = yellowCone->getPosition().distanceTo(state1->getPosition());
                double distanceToBlue = blueCone->getPosition().distanceTo(state1->getPosition());

                return abs(distanceToBlue - distanceToYellow);
            }

        } // path_planner
    } // search
} // heuristics