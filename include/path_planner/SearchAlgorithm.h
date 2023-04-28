//
// Created by carlostojal on 28/04/2023.
//

#ifndef DELAUNAY_PATH_PLANNER_CORE_SEARCHALGORITHM_H
#define DELAUNAY_PATH_PLANNER_CORE_SEARCHALGORITHM_H

#include <list>
#include <memory>
#include <path_planner/State.h>

namespace path_planner {

    class SearchAlgorithm {
        protected:
            std::shared_ptr<path_planner::State> initialState = nullptr;
            std::shared_ptr<path_planner::State> goalState = nullptr;

        public:
            void setInitialState(std::shared_ptr<path_planner::State> initial);
            void setGoalState(std::shared_ptr<path_planner::State> goal);

            virtual std::list<std::shared_ptr<path_planner::State>> search() = 0;
    };

} // path_planner

#endif //DELAUNAY_PATH_PLANNER_CORE_SEARCHALGORITHM_H
