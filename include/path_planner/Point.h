//
// Created by carlostojal on 27/04/2023.
//

#ifndef DELAUNAY_PATH_PLANNER_CORE_POINT_H
#define DELAUNAY_PATH_PLANNER_CORE_POINT_H

#include <stdexcept>
#include <math.h>

namespace path_planner {

    class Point {

        private:
            double x;
            double y;
            double theta;

            bool xSet = false;
            bool ySet = false;
            bool thetaSet = false;

        public:
            Point();

            double getX() const;
            void setX(double x);

            double getY() const;
            void setY(double y);

            double getTheta() const;
            void setTheta(double theta);

            double distanceTo(const Point& other);
    };

} // path_planner

#endif //DELAUNAY_PATH_PLANNER_CORE_POINT_H
