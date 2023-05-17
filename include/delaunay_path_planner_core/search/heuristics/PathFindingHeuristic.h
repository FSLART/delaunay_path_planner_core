//
// Created by carlostojal on 06-05-2023.
//

#ifndef DELAUNAY_PATH_PLANNER_CORE_PATHFINDINGHEURISTIC_H
#define DELAUNAY_PATH_PLANNER_CORE_PATHFINDINGHEURISTIC_H

#include "Heuristic.h"
#include "delaunay_path_planner_core/Cone.h"
#include <queue>
#include <thread>

namespace path_planner {
    namespace search {
        namespace heuristics {

            class PathFindingHeuristic : Heuristic {
                public:
                    double compute(const std::shared_ptr<path_planner::State>& state1,
                                   const std::shared_ptr<path_planner::State>& state2,
                                   const std::shared_ptr<path_planner::State>& goalState,
                                   double currentGCost = 0) override;
                    /*!
                     * @brief Find the closest cone of a given color starting from a state.
                     * It uses breadth first search to find the closest yellow and blue nodes, and the distance differences
                     * are used as heuristic.
                     *
                     * @param c The color of the cone to search
                     * @param initialState The initial state of the search
                     * @return A pointer to the found state
                     */
                    std::shared_ptr<path_planner::State> coneFindingSearch(cone_color_t c,
                                                                           std::shared_ptr<path_planner::State> initialState);
                /*!
                 * \brief Method intended to be called from a thread. Find the closest cone of a given color.
                 *
                 * @param instance The instance of the PathFindHeuristic to search on.
                 * @param c The color to search for.
                 * @param initialState The initial state of the search.
                 * @return A pointer to the found state.
                 */
                friend void findClosestConeRoutine(PathFindingHeuristic *instance,
                                                   cone_color_t c,
                                                   std::shared_ptr<path_planner::State> initialState,
                                                    std::shared_ptr<path_planner::State>* found);
            };

        } // path_planner
    } // search
} // heuristics

#endif //DELAUNAY_PATH_PLANNER_CORE_PATHFINDINGHEURISTIC_H
