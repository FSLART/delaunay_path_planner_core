//
// Created by 2211006 on 27/04/2023.
//

#ifndef DELAUNAY_PATH_PLANNER_CORE_TRIANGLE_H
#define DELAUNAY_PATH_PLANNER_CORE_TRIANGLE_H

#include <path_planner/Point.h>

namespace path_planner {

    struct Triangle {
        path_planner::Point a, b, c;
        Triangle(Point a, Point b, Point c) : a(a), b(b), c(c) {}
    };

} // path_planner

#endif //DELAUNAY_PATH_PLANNER_CORE_TRIANGLE_H
