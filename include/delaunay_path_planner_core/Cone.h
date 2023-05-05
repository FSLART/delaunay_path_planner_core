//
// Created by carlostojal on 29-04-2023.
//

#ifndef DELAUNAY_PATH_PLANNER_CORE_CONE_H
#define DELAUNAY_PATH_PLANNER_CORE_CONE_H

#include "delaunay_path_planner_core/Point.h"

namespace path_planner {

    enum cone_color_t {
        BLUE,
        YELLOW,
        ORANGE,
        UNKNOWN
    };

    class Cone : public path_planner::Point {
        private:
            cone_color_t color;
            bool colorSet = false;

        public:
            Cone();
            cone_color_t getColor() const;
            void setColor(cone_color_t color);

    };

} // path_planner

#endif //DELAUNAY_PATH_PLANNER_CORE_CONE_H
