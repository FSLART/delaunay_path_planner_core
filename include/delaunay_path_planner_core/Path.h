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
            /*! @brief List of states in order. */
            std::list<std::shared_ptr<path_planner::State>> states;
            /*! @brief Cost of the path. */
            float cost = 0;

        public:
            Path();
            ~Path();

            /*! @brief Add a state to the path.
             *
             * @param s The state to add.
             * */
            void addState(const std::shared_ptr<path_planner::State>& s, float cost = 0);
            void prependState(const std::shared_ptr<path_planner::State>& s, float cost = 0);
    };

} // path_planner

#endif //DELAUNAY_PATH_PLANNER_CORE_PATH_H
