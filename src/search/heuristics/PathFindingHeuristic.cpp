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

            double PathFindingHeuristic::compute(const std::shared_ptr<path_planner::State>& state1,
                                                 const std::shared_ptr<path_planner::State>& state2,
                                                 const std::shared_ptr<path_planner::State>& goalState,
                                                 double currentGCost) {

            }

        } // path_planner
    } // search
} // heuristics