//
// Created by carlostojal on 27/04/2023.
//

#include <delaunay_path_planner_core/Environment.h>

namespace path_planner {

    Environment::Environment() {
        this->cones = std::set<std::shared_ptr<path_planner::State>>();
    }

    Environment::~Environment() {
        // free the car state pointer
        this->carState.reset();
    }

    std::set<std::shared_ptr<path_planner::State>> Environment::getCones() const {
        return this->cones;
    }

    std::shared_ptr<path_planner::State> Environment::getCarState() const {
        return this->carState;
    }

    void Environment::setCarState(const std::shared_ptr<path_planner::State>& state) {
        this->carState = state;
        this->carState->setOccupancy(CAR_OCCUPANCY);
    }

    void Environment::addCone(const path_planner::Cone& cone) {
        std::shared_ptr<path_planner::State> coneState = std::make_shared<path_planner::State>();
        coneState->setPosition(cone);
        switch (cone.getColor()) {
            case BLUE:
                coneState->setOccupancy(BLUE_CONE_OCCUPANCY);
                break;
            case YELLOW:
                coneState->setOccupancy(YELLOW_CONE_OCCUPANCY);
                break;
            case ORANGE:
                coneState->setOccupancy(ORANGE_CONE_OCCUPANCY);
                break;
            case UNKNOWN:
                coneState->setOccupancy(UNKNOWN_CONE_OCCUPANCY);
                break;
        }
        this->cones.insert(coneState);
    }

    std::shared_ptr<State> Environment::generateGraph() {

        std::unordered_map<path_planner::Point,std::shared_ptr<path_planner::State>> statesByPosition;

        Delaunay dt;

        // insert the car position into the triangulation
        dt.insert(this->carState->getPosition().getAsCGALPoint());
        statesByPosition[this->carState->getPosition()] = this->carState;

        // insert the goal position
        dt.insert(this->goalState->getPosition().getAsCGALPoint());
        statesByPosition[this->goalState->getPosition()] = this->goalState;

        // insert all cones into the triangulation
        for(auto iter : this->cones) {
            statesByPosition[iter->getPosition()] = iter;
            dt.insert(iter->getPosition().getAsCGALPoint());
        }

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
            K::Point_2 p1 = iter->first->vertex((iter->second + 1) % 3)->point();
            K::Point_2 p2 = iter->first->vertex((iter->second + 2) % 3)->point();

            // build points just to perform map lookup
            path_planner::Point p1Query = path_planner::Point(p1.x(), p1.y());
            path_planner::Point p2Query = path_planner::Point(p2.x(), p2.y());

            // add p2 as neighbor of p1
            statesByPosition[p1Query]->addNeighbor(statesByPosition[p2Query]);
            // add p1 as neightbor of p2
            statesByPosition[p2Query]->addNeighbor(statesByPosition[p1Query]);

        }

        // return the current car state as an entry point
        return this->carState;
    }

    std::shared_ptr<path_planner::State> Environment::getGoalState() const {
        return this->goalState;
    }

    void Environment::setGoalState(const std::shared_ptr<path_planner::State> &state) {
        this->goalState = state;
    }

} // path_planner