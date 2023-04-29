//
// Created by carlostojal on 27/04/2023.
//

#ifndef DELAUNAY_PATH_PLANNER_CORE_POINT_H
#define DELAUNAY_PATH_PLANNER_CORE_POINT_H

#include <stdexcept>
#include <math.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Delaunay_triangulation_2<K> Delaunay;

namespace path_planner {

    class Point {

        private:
            double x;
            double y;
            double theta;

            bool xSet = false;
            bool ySet = false;
            bool thetaSet = false;

            K::Point_2 cgalPoint;

            void tryInitCgalPoint(double x, double y);


        public:
            Point();
            Point(double x, double y);
            Point(double x, double y, double theta);

            double getX() const;
            void setX(double x);

            double getY() const;
            void setY(double y);

            double getTheta() const;
            void setTheta(double theta);

            K::Point_2 getAsCGALPoint() const;

            double distanceTo(const Point& other);
    };

} // path_planner

#endif //DELAUNAY_PATH_PLANNER_CORE_POINT_H
