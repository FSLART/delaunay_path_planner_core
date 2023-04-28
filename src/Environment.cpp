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

        // TODO: this graph generation method will not work

        std::set<std::shared_ptr<path_planner::State>> stateSet;

        // add the current state to the set
        stateSet.insert(this->carState);

        // add all cone states to the set
        for(auto & cone : this->cones)
            stateSet.insert(std::make_shared<path_planner::State>(cone));

        // connect all states
        for(auto stateIter = stateSet.begin(); stateIter != stateSet.end(); stateIter++) {
            for(const auto & stateIter1 : stateSet) {
                (*stateIter)->addNeighbor(stateIter1);
            }
        }

        // subdivide
        std::set<std::shared_ptr<path_planner::State>> midPointsSet;
        for(const auto & stateIter : stateSet) {
            // get neighbor set
            std::set neighbors = stateIter->getNeighbors();
            // create intermediate state between all neighbors
            for(const auto & neighbor : neighbors) {
                midPointsSet.insert(stateIter->createIntermediate(neighbor));
            }
        }

        // connect midpoints among them
        for(auto midIter = midPointsSet.begin(); midIter != midPointsSet.end(); midIter++) {
            for(const auto & midIter1 : midPointsSet)
                (*midIter)->addNeighbor(midIter1);
        }

        // return the current car state as an entry point
        return this->carState;
    }

} // path_planner