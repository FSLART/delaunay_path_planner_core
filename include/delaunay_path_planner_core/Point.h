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
            float x;
            /*! @brief Y value */
            float y;
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
            Point(const float& x, const float& y);
            /*! @brief Initialize a point with position and yaw angle.
             *
             * @param x X value
             * @param y Y value
             * @param theta Yaw angle
             */
            Point(const float& x, const float& y, const double& theta);
            /*! @brief Initialize a point with a CGAL point.
             *
             * @param p CGAL point.
             */
            Point(const K::Point_2& p);

            /*! @brief Get X value */
            float getX() const;

            /*! @brief Set X value */
            void setX(const float& x);

            /*! @brief Get Y value */
            float getY() const;
            /*! @brief Set Y value */
            void setY(const float& y);

            /*! @brief Get yaw angle */
            double getTheta() const;
            /*! @brief Set yaw angle */
            void setTheta(const double& theta);

            /*! @brief Get the Point as a CGAL point */
            K::Point_2 getAsCGALPoint() const;

            /*! @brief Compute the distance from this point instance to another */
            double distanceTo(const Point& other);

            /*! \brief Get a string representation of this instance. */
            std::string _str_();

            /*! @brief Compare this Point instance with a CGAL point by coordinates */
            bool operator==(const K::Point_2& other) const;

            bool operator!=(const K::Point_2& other) const;

            /*! @brief Compare this Point instance with another by coordinates */
            bool operator==(const Point& other) const;

            bool operator!=(const Point& other) const;
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
