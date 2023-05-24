//
// Created by carlostojal on 24-05-2023.
//

#ifndef DELAUNAY_PATH_PLANNER_CORE_BREADTHFIRSTSEARCH_H
#define DELAUNAY_PATH_PLANNER_CORE_BREADTHFIRSTSEARCH_H

#include <delaunay_path_planner_core/search/SearchAlgorithm.h>
#include <queue>
#include <unordered_set>
#include <unordered_map>

namespace path_planner {
    namespace search {

        class BreadthFirstSearch : SearchAlgorithm {

            public:
                path_planner::Path search() override;
        };

    } // path_planner
} // search

#endif //DELAUNAY_PATH_PLANNER_CORE_BREADTHFIRSTSEARCH_H
