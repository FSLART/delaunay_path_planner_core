//
// Created by carlostojal on 27/04/2023.
//

#include <delaunay_path_planner_core/Environment.h>

namespace path_planner {

    Environment::Environment() {
        this->cones = std::vector<path_planner::Cone>();
    }

    std::vector<path_planner::Cone> Environment::getCones() const {
        return this->cones;
    }

    path_planner::State Environment::getCarState() const {
        return *this->carState;
    }

    void Environment::setCarState(const path_planner::State& state) {
        this->carState = std::make_shared<path_planner::State>(state);
        this->carState->setOccupancy(CAR_OCCUPANCY);
    }

    void Environment::addCone(const path_planner::Cone& cone) {
        this->cones.push_back(cone);
    }

    std::shared_ptr<State> Environment::generateGraph() {

        Delaunay dt;

        // insert the car position into the triangulation
        dt.insert(this->carState->getPosition().getAsCGALPoint());

        // insert all cones into the triangulation
        for(auto iter : this->cones)
            dt.insert(iter.getAsCGALPoint());

        auto initial_edges = dt.finite_edges();

        // re-triangulate with the midpoints
        for(auto & pair : initial_edges) {
            // get the vertices of this edge
            K::Point_2 p1 = pair.first->vertex((pair.second + 1) % 3)->point();
            K::Point_2 p2 = pair.first->vertex((pair.second + 2) % 3)->point();

            // add the midpoint to the triangulation
            dt.insert(K::Point_2((p1.x() + p2.x()) / 2, (p1.y() + p2.y()) / 2));
        }

        // create the relations
        for(auto iter = dt.finite_edges_begin(); iter != dt.finite_edges_end(); iter++) {
            // get the vertices of this edge
            K::Point_2 p1 = iter->first->vertex((iter->second+1)%3)->point();
            K::Point_2 p2 = iter->first->vertex((iter->second+2)%3)->point();

            // TODO: build a hashmap or hashtable to get cones by coordinates

            Point p1_point = path_planner::Point(p1);
            Point p2_point = path_planner::Point(p2);

            // check if some of these are the car state
            if(p1 == this->carState->getPosition().getAsCGALPoint()) {
                std::shared_ptr<State> p2_state = std::make_shared<State>(p2_point);
                this->carState->addNeighbor(p2_state);
                p2_state->addNeighbor(this->carState);
                continue;
            } else if(p2 == this->carState->getPosition().getAsCGALPoint()) {
                std::shared_ptr<State> p1_state = std::make_shared<State>(p1_point);
                this->carState->addNeighbor(p1_state);
                p1_state->addNeighbor(this->carState);
                continue;
            } else {

                Cone point1Cone;
                bool point1ConeExists = false;

                Cone point2Cone;
                bool point2ConeExists = false;

                // search p1 and p2 in cones
                for(auto coneIter : this->cones) {

                    if(coneIter == p1_point) {
                        point1ConeExists = true;
                        point1Cone = coneIter;
                    }

                    if(coneIter == p2_point) {
                        point2ConeExists = true;
                        point2Cone = coneIter;
                    }
                }

                // none of these vertices are the car state
                std::shared_ptr<State> p1_state = std::make_shared<State>(p1);
                std::shared_ptr<State> p2_state = std::make_shared<State>(p2);

                if(point1ConeExists)
                    p1_state->setOccupancy(point1Cone.getColor() == BLUE ? BLUE_CONE_OCCUPANCY : YELLOW_CONE_OCCUPANCY);

                if(point2ConeExists)
                    p2_state->setOccupancy(point2Cone.getColor() == BLUE ? BLUE_CONE_OCCUPANCY : YELLOW_CONE_OCCUPANCY);

                p1_state->addNeighbor(p2_state);
                p2_state->addNeighbor(p1_state);
            }
        }

        // return the current car state as an entry point
        return this->carState;
    }

} // path_planner