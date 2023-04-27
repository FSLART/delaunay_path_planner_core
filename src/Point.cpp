//
// Created by carlostojal on 27/04/2023.
//

#include <path_planner/Point.h>

namespace path_planner {

    double Point::getX() const {
        if(!this->xSet)
            throw std::runtime_error("Point x value not set!");

        return x;
    }

    void Point::setX(double x) {
        this->xSet = true;
        this->x = x;
    }

    double Point::getY() const {
        if(!this->ySet)
            throw std::runtime_error("Point y value not set!");

        return y;
    }

    void Point::setY(double y) {
        this->ySet = true;
        this->y = y;
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
} // path_planner