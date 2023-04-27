//
// Created by 2211006 on 27/04/2023.
//

#ifndef DELAUNAY_PATH_PLANNER_CORE_STATE_H
#define DELAUNAY_PATH_PLANNER_CORE_STATE_H

#include <path_planner/Point.h>
#include "Point.h"

namespace path_planner {

    class State {
        private:
            path_planner::Point position;

        public:
            State();

            path_planner::Point getPosition();
            void setPosition(path_planner::Point position);
    };

} // path_planner

#endif //DELAUNAY_PATH_PLANNER_CORE_STATE_H
