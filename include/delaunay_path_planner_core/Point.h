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
            /*! @brief X value */
            long int x;
            /*! @brief Y value */
            long int y;
            /*! @brief Yaw angle value */
            double theta;

            bool xSet = false;
            bool ySet = false;
            bool thetaSet = false;

            /*! @brief The point as used by CGAL, the library which computes the triangulation. */
            K::Point_2 cgalPoint;

            /*! @brief Initialize a CGAL point with a pair of coordinates. */
            void tryInitCgalPoint(const double& x, const double& y);


        public:
            Point();

            /*! @brief Initialize a point with position.
             *
             * @param x X value
             * @param y Y value
             * */
            Point(const long int& x, const long int& y);
            /*! @brief Initialize a point with position and yaw angle.
             *
             * @param x X value
             * @param y Y value
             * @param theta Yaw angle
             */
            Point(const long int& x, const long int& y, const double& theta);
            /*! @brief Initialize a point with a CGAL point.
             *
             * @param p CGAL point.
             */
            Point(const K::Point_2& p);

            /*! @brief Get X value */
            long int getX() const;

            /*! @brief Set X value */
            void setX(const long int& x);

            /*! @brief Get Y value */
            long int getY() const;
            /*! @brief Set Y value */
            void setY(const long int& y);

            /*! @brief Get yaw angle */
            double getTheta() const;
            /*! @brief Set yaw angle */
            void setTheta(const double& theta);

            /*! @brief Get the Point as a CGAL point */
            K::Point_2 getAsCGALPoint() const;

            /*! @brief Compute the distance from this point instance to another */
            double distanceTo(const Point& other);

            /*! @brief Compare this Point instance with a CGAL point by coordinates */
            bool operator==(const K::Point_2& other) const;

            /*! @brief Compare this Point instance with another by coordinates */
            bool operator==(const Point& other) const;
    };

} // path_planner

/*! @brief Custom hash function of a Point involving its coordinates */
template <>
struct std::hash<path_planner::Point> {
    std::size_t operator()(path_planner::Point const& p) const noexcept
    {
        std::size_t h1 = std::hash<long int>{}(p.getX());
        std::size_t h2 = std::hash<long int>{}(p.getY());
        return h1 ^ (h2 << 1);
    }
};

/*! @brief Custom hash function of a CGAL point involving its coordinates */
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
