//
// Created by carlostojal on 17-05-2023.
//

#ifndef DELAUNAY_PATH_PLANNER_CORE_UNIFORMCOST_H
#define DELAUNAY_PATH_PLANNER_CORE_UNIFORMCOST_H

#include <delaunay_path_planner_core/search/InformedSearchAlgorithm.h>
#include <delaunay_path_planner_core/search/heuristics/ClosestConeFindingHeuristic.h>

namespace path_planner {
    namespace search {

        class UniformCost : InformedSearchAlgorithm<path_planner::search::heuristics::ClosestConeFindingHeuristic> {

            public:
                path_planner::Path search() override;
        };

    } // path_planner
} // search

#endif //DELAUNAY_PATH_PLANNER_CORE_UNIFORMCOST_H
