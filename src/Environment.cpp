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

    void Environment::addCone(const path_planner::Point& cone) {
        this->cones.push_back(cone);
    }

    std::shared_ptr<State> Environment::generateGraph() {

        Delaunay dt;

        // insert the car position into the triangulation
        dt.insert(this->carState->getPosition().getAsCGALPoint());

        // insert all cones into the triangulation
        for(auto iter : this->cones)
            dt.insert(iter.getAsCGALPoint());

        // iterate edges
        for(auto iter = dt.finite_edges_begin(); iter != dt.finite_edges_end(); ++iter) {
            // get the vertices of this edge
            K::Point_2 p1 = iter->first->vertex((iter->second+1)%3)->point();
            K::Point_2 p2 = iter->first->vertex((iter->second+2)%3)->point();

            // check if some of these are the car state
            if(p1 == this->carState->getPosition().getAsCGALPoint()) {
                std::shared_ptr<State> p2_state = std::make_shared<State>(p2);
                this->carState->addNeighbor(p2_state);
                p2_state->addNeighbor(this->carState);
                continue;
            } else if(p2 == this->carState->getPosition().getAsCGALPoint()) {
                std::shared_ptr<State> p1_state = std::make_shared<State>(p1);
                this->carState->addNeighbor(p1_state);
                p1_state->addNeighbor(this->carState);
                continue;
            } else {

                // none of these vertices are the car state
                std::shared_ptr<State> p1_state = std::make_shared<State>(p1);
                std::shared_ptr<State> p2_state = std::make_shared<State>(p2);

                p1_state->addNeighbor(p2_state);
                p2_state->addNeighbor(p1_state);
            }
        }

        // return the current car state as an entry point
        return this->carState;
    }

} // path_planner