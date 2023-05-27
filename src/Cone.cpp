//
// Created by carlostojal on 29-04-2023.
//

#include <delaunay_path_planner_core/Cone.h>

namespace path_planner {

    Cone::Cone() : Point() {
        this->color = UNKNOWN;
    }

    Cone::Cone(const float& x, const float& y) : Point(x, y) {
        this->color = UNKNOWN;
    }

    Cone::Cone(const float& x, const float& y, const cone_color_t& c) : Cone(x, y) {
        this->color = c;
    }

    cone_color_t Cone::getColor() const {
        return color;
    }

    void Cone::setColor(cone_color_t color) {
        this->color = color;
        this->colorSet = true;
    }


} // path_planner