//
// Created by carlostojal on 27/04/2023.
//

#ifndef DELAUNAY_PATH_PLANNER_CORE_POINT_H
#define DELAUNAY_PATH_PLANNER_CORE_POINT_H

#include <stdexcept>
#include <math.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;

namespace path_planner {

    class Point {

        protected:
            long int x;
            long int y;
            double theta;

            bool xSet = false;
            bool ySet = false;
            bool thetaSet = false;

            K::Point_2 cgalPoint;

            void tryInitCgalPoint(double x, double y);


        public:
            Point();
            Point(long int x, long int y);
            Point(long int x, long int y, double theta);
            Point(K::Point_2 p);

            long int getX() const;
            void setX(long int x);

            long int getY() const;
            void setY(long int y);

            double getTheta() const;
            void setTheta(double theta);

            K::Point_2 getAsCGALPoint() const;

            double distanceTo(const Point& other);

            bool operator==(const K::Point_2& other) const;

            bool operator==(const Point& other) const;
    };

} // path_planner

// hash function of Point
template <>
struct std::hash<path_planner::Point> {
    std::size_t operator()(path_planner::Point const& p) const noexcept
    {
        std::size_t h1 = std::hash<long int>{}(p.getX());
        std::size_t h2 = std::hash<long int>{}(p.getY());
        return h1 ^ (h2 << 1);
    }
};

// custom hash function of cgal point
template <>
struct std::hash<K::Point_2> {
    std::size_t operator()(K::Point_2 const& p) const noexcept
    {
        std::size_t h1 = std::hash<long int>{}(p.x());
        std::size_t h2 = std::hash<long int>{}(p.y());
        return h1 ^ (h2 << 1);
    }
};

#endif //DELAUNAY_PATH_PLANNER_CORE_POINT_H
