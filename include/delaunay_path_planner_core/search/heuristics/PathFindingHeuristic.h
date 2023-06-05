//
// Created by carlostojal on 06-05-2023.
//

#ifndef DELAUNAY_PATH_PLANNER_CORE_PATHFINDINGHEURISTIC_H
#define DELAUNAY_PATH_PLANNER_CORE_PATHFINDINGHEURISTIC_H

#include "Heuristic.h"
#include <lart_common/Cone.h>
#include <delaunay_path_planner_core/search/BreadthFirstSearch.h>
#include <queue>
#include <thread>

namespace path_planner {
    namespace search {
        namespace heuristics {

            class PathFindingHeuristic : Heuristic {
                public:
                    double compute(const std::shared_ptr<lart_common::State>& state1,
                                   const std::shared_ptr<lart_common::State>& state2,
                                   const std::shared_ptr<lart_common::State>& goalState,
                                   double currentGCost = 0) override;

                /*!
                 * \brief Method intended to be called from a thread. Find the closest cone of a given color.
                 *
                 * @param instance The instance of the PathFindHeuristic to search on.
                 * @param c The color to search for.
                 * @param initialState The initial state of the search.
                 * @return A pointer to the found state.
                 */
                    static void findClosestConeRoutine(lart_common::occupancy_type_t t,
                                                       const std::shared_ptr<lart_common::State>& initialState,
                                                       std::shared_ptr<lart_common::State>& found);
            };

        } // path_planner
    } // search
} // heuristics

#endif //DELAUNAY_PATH_PLANNER_CORE_PATHFINDINGHEURISTIC_H
