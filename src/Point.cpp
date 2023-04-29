//
// Created by carlostojal on 27/04/2023.
//

#include <path_planner/Point.h>

namespace path_planner {

    Point::Point() {

    }

    Point::Point(double x, double y) {
        this->setX(x);
        this->setY(y);
        this->cgalPoint = K::Point_2(x, y);
    }

    Point::Point(double x, double y, double theta) {
        this->setX(x);
        this->setY(y);
        this->setTheta(theta);
        this->cgalPoint = K::Point_2(x, y);
    }

    double Point::getX() const {
        if(!this->xSet)
            throw std::runtime_error("Point x value not set!");

        return x;
    }

    void Point::setX(double x) {
        this->xSet = true;
        this->x = x;
        this->tryInitCgalPoint(x, this->y);
    }

    double Point::getY() const {
        if(!this->ySet)
            throw std::runtime_error("Point y value not set!");

        return y;
    }

    void Point::setY(double y) {
        this->ySet = true;
        this->y = y;
        this->tryInitCgalPoint(x, this->y);
    }

    double Point::getTheta() const {
        if(!this->thetaSet)
            throw std::runtime_error("Point theta value not set!");

        return theta;
    }

    void Point::setTheta(double theta) {
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

    void Point::tryInitCgalPoint(double x, double y) {
        if(this->xSet && this->ySet)
            this->cgalPoint = K::Point_2(x, y);
    }
} // path_planner