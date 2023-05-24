//
// Created by carlostojal on 27/04/2023.
//

#include <delaunay_path_planner_core/Point.h>

namespace path_planner {

    Point::Point() {

    }

    Point::Point(const long int& x, const long int& y) {
        this->setX(x);
        this->setY(y);
        this->cgalPoint = K::Point_2(x, y);
    }

    Point::Point(const long int& x, const long int& y, const double& theta) {
        this->setX(x);
        this->setY(y);
        this->setTheta(theta);
        this->cgalPoint = K::Point_2(x, y);
    }

    Point::Point(const K::Point_2& p) {
        this->setX(p.x());
        this->setY(p.y());
    }

    long int Point::getX() const {
        if(!this->xSet)
            throw std::runtime_error("Point x value not set!");

        return x;
    }

    void Point::setX(const long int& x) {
        this->xSet = true;
        this->x = x;
        this->tryInitCgalPoint(x, this->y);
    }

    long int Point::getY() const {
        if(!this->ySet)
            throw std::runtime_error("Point y value not set!");

        return y;
    }

    void Point::setY(const long int& y) {
        this->ySet = true;
        this->y = y;
        this->tryInitCgalPoint(x, this->y);
    }

    double Point::getTheta() const {
        if(!this->thetaSet)
            throw std::runtime_error("Point theta value not set!");

        return theta;
    }

    void Point::setTheta(const double& theta) {
        this->thetaSet = true;
        this->theta = theta;
    }

    K::Point_2 Point::getAsCGALPoint() const {
        if(!this->xSet or !this->ySet)
            throw std::runtime_error("Point not completely set!");

        return cgalPoint;
    }

    double Point::distanceTo(const Point &other) {
        return std::sqrt(std::pow(std::abs(this->x - other.x), 2) +
            std::pow(std::abs(this->y - other.y), 2));
    }

    void Point::tryInitCgalPoint(const double& x, const double& y) {
        if(this->xSet && this->ySet)
            this->cgalPoint = K::Point_2(x, y);
    }

    bool Point::operator==(const Point& other) const {
        return this->x == other.x && this->y == other.y;
    }

    bool Point::operator==(const K::Point_2& other) const {
        return this->getAsCGALPoint() == other;
    }

} // path_planner