//
// Created by carlostojal on 27/04/2023.
//

#include "path_planner/Environment.h"

namespace path_planner {

    Environment::Environment() {
        this->cones = std::vector<path_planner::Point>();
    }

    std::vector<path_planner::Point> Environment::getCones() const {
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

        std::set<std::shared_ptr<path_planner::State>> stateSet;

        // add the current state to the set
        stateSet.insert(this->carState);

        // add all cone states to the set
        for(auto coneIter = this->cones.begin(); coneIter != this->cones.end(); coneIter++)
            stateSet.insert(std::make_shared<path_planner::State>(*coneIter));

        // connect all states
        for(auto stateIter = stateSet.begin(); stateIter != stateSet.end(); stateIter++) {
            for(auto stateIter1 = stateSet.begin(); stateIter1 != stateSet.end(); stateIter1++) {
                (*stateIter)->addNeighbor(*stateIter1);
            }
        }

        // subdivide
        for(auto stateIter = stateSet.begin(); stateIter != stateSet.end(); stateIter++) {
            // get neighbor set
            std::set neighbors = (*stateIter)->getNeighbors();
            // create intermediate state between all neighbors
            for(auto neighborIter = neighbors.begin(); neighborIter != neighbors.end(); neighborIter++) {
                (*stateIter)->createIntermediate(*neighborIter);
            }
        }

        // return the current car state as an entry point
        return this->carState;
    }

} // path_planner