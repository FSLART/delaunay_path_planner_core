//
// Created by carlostojal on 06-05-2023.
//

#ifndef DELAUNAY_PATH_PLANNER_CORE_BREADTHFIRSTSEARCH_H
#define DELAUNAY_PATH_PLANNER_CORE_BREADTHFIRSTSEARCH_H

#include <delaunay_path_planner_core/search/SearchAlgorithm.h>
#include <delaunay_path_planner_core/Path.h>
#include <queue>

namespace path_planner {
    namespace search {

        class BreadthFirstSearch : SearchAlgorithm {

            public:
                path_planner::Path search() override;
        };

    } // path_planner
} // search

#endif //DELAUNAY_PATH_PLANNER_CORE_BREADTHFIRSTSEARCH_H
