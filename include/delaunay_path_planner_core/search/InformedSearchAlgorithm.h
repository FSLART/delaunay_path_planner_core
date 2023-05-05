//
// Created by carlostojal on 29-04-2023.
//

#ifndef DELAUNAY_PATH_PLANNER_CORE_INFORMEDSEARCHALGORITHM_H
#define DELAUNAY_PATH_PLANNER_CORE_INFORMEDSEARCHALGORITHM_H

#include "SearchAlgorithm.h"

namespace path_planner::search {

    template <typename HeuristicT>
    class InformedSearchAlgorithm : protected SearchAlgorithm {

        protected:
            HeuristicT heuristic;
    };

} // search

#endif //DELAUNAY_PATH_PLANNER_CORE_INFORMEDSEARCHALGORITHM_H
