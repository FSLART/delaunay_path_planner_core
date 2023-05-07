//
// Created by carlostojal on 28/04/2023.
//

#ifndef DELAUNAY_PATH_PLANNER_CORE_GREEDYSEARCH_H
#define DELAUNAY_PATH_PLANNER_CORE_GREEDYSEARCH_H

#include <delaunay_path_planner_core/search/InformedSearchAlgorithm.h>
#include <delaunay_path_planner_core/search/heuristics/GreedyHeuristic.h>

namespace path_planner::search {

    class GreedySearch : InformedSearchAlgorithm<path_planner::search::heuristics::GreedyHeuristic> {

        public:
            path_planner::Path search() override;
    };

} // path_planner

#endif //DELAUNAY_PATH_PLANNER_CORE_GREEDYSEARCH_H
