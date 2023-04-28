//
// Created by carlostojal on 28/04/2023.
//

#ifndef DELAUNAY_PATH_PLANNER_CORE_GREEDYSEARCH_H
#define DELAUNAY_PATH_PLANNER_CORE_GREEDYSEARCH_H

#include <path_planner/SearchAlgorithm.h>

namespace path_planner {

    class GreedySearch : SearchAlgorithm {

        public:
            std::list<std::shared_ptr<path_planner::State>> search() override;
    };

} // path_planner

#endif //DELAUNAY_PATH_PLANNER_CORE_GREEDYSEARCH_H
