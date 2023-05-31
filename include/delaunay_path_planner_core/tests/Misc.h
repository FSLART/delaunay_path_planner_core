//
// Created by carlostojal on 26-05-2023.
//

#ifndef DELAUNAY_PATH_PLANNER_CORE_MISC_H
#define DELAUNAY_PATH_PLANNER_CORE_MISC_H

#include <delaunay_path_planner_core/Environment.h>

namespace path_planner::tests {
    class Misc {

        public:
            static path_planner::Environment generateStraightSegment();
            static path_planner::Environment generateCurvedSegment();
            static path_planner::Environment generateInconsistentSegment();
    };
}
// TODO: create straight track environment

// TODO: create curved track environment

#endif //DELAUNAY_PATH_PLANNER_CORE_MISC_H