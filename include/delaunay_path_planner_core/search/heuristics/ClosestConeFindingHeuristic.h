//
// Created by carlostojal on 17-05-2023.
//

#ifndef DELAUNAY_PATH_PLANNER_CORE_CLOSESTCONEFINDINGHEURISTIC_H
#define DELAUNAY_PATH_PLANNER_CORE_CLOSESTCONEFINDINGHEURISTIC_H

#include <delaunay_path_planner_core/search/heuristics/Heuristic.h>

namespace path_planner {
    namespace search {
        namespace heuristics {

            class ClosestConeFindingHeuristic : path_planner::search::heuristics::Heuristic {
                double compute(const std::shared_ptr<path_planner::State>& state1,
                               const std::shared_ptr<path_planner::State>& state2,
                               const std::shared_ptr<path_planner::State>& goalState, double currentGCost=0) override;
            };

        } // path_planner
    } // search
} // heuristics

#endif //DELAUNAY_PATH_PLANNER_CORE_CLOSESTCONEFINDINGHEURISTIC_H
