//
// Created by carlostojal on 05-05-2023.
//

#ifndef DELAUNAY_PATH_PLANNER_CORE_PATH_H
#define DELAUNAY_PATH_PLANNER_CORE_PATH_H

#include <list>
#include <memory>
#include <delaunay_path_planner_core/State.h>

namespace path_planner {

    class Path {

        private:
            std::list<std::shared_ptr<path_planner::State>> states;
            float cost = 0;

        public:
            Path();
            ~Path();

            void addState(const std::shared_ptr<path_planner::State>& s);
    };

} // path_planner

#endif //DELAUNAY_PATH_PLANNER_CORE_PATH_H
