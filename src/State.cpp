//
// Created by 2211006 on 27/04/2023.
//

#include "../include/path_planner/State.h"

namespace path_planner {

    State::State() {
        this->position = path_planner::Point();
    }

    path_planner::Point State::getPosition() {
        return this->position;
    }

    void State::setPosition(path_planner::Point position) {
        this->position = position;
    }

} // path_planner