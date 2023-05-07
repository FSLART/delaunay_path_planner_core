//
// Created by carlostojal on 28/04/2023.
//

#ifndef DELAUNAY_PATH_PLANNER_CORE_SEARCHALGORITHM_H
#define DELAUNAY_PATH_PLANNER_CORE_SEARCHALGORITHM_H

#include <list>
#include <memory>
#include <delaunay_path_planner_core/State.h>
#include <delaunay_path_planner_core/search/heuristics/Heuristic.h>
#include <delaunay_path_planner_core/Path.h>

namespace path_planner::search {

    class SearchAlgorithm {
        protected:
            std::shared_ptr<path_planner::State> initialState = nullptr;
            std::shared_ptr<path_planner::State> goalState = nullptr;
            size_t maxIterations = 99;
            // TODO: use custom comparator passed to the search algorithm
            std::function<bool(const std::shared_ptr<path_planner::State>&, const std::shared_ptr<path_planner::State>&)> cmp;

        public:
            void setInitialState(std::shared_ptr<path_planner::State> initial);
            void setGoalState(std::shared_ptr<path_planner::State> goal);

            size_t getMaxIterations() const;
            void setMaxIterations(size_t n_iterations);

            virtual path_planner::Path search() = 0;
    };

} // path_planner

#endif //DELAUNAY_PATH_PLANNER_CORE_SEARCHALGORITHM_H
