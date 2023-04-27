//
// Created by carlostojal on 27/04/2023.
//

#include "path_planner/Environment.h"

namespace path_planner {

    Environment::Environment() {
        this->cones = std::list<path_planner::Point>();
    }

    std::list<path_planner::Point> Environment::getCones() const {
        return this->cones;
    }

    path_planner::State Environment::getCarState() const {
        return *this->carState;
    }

    void Environment::setCarState(const path_planner::State& state) {
        this->carState = std::make_shared<path_planner::State>(state);
    }

    void Environment::addCone(path_planner::Point cone) {
        this->cones.push_back(cone);
    }

    std::shared_ptr<State> Environment::generateGraph() {
        /* TODO
         * - triangulate
         * - set the pointers between states
         */
        return this->carState;
    }

} // path_planner