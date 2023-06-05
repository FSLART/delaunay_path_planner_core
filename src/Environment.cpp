//
// Created by carlostojal on 27/04/2023.
//

#include <delaunay_path_planner_core/Environment.h>

namespace path_planner {

    Environment::Environment() {
        this->cones = std::set<std::shared_ptr<lart_common::State>>();
    }

    Environment::~Environment() {
        // free the car state pointer
        this->carState.reset();
    }

    std::set<std::shared_ptr<lart_common::State>> Environment::getCones() const {
        return this->cones;
    }

    std::shared_ptr<lart_common::State> Environment::getCarState() const {
        return this->carState;
    }

    void Environment::setCarState(const std::shared_ptr<lart_common::State>& state) {
        this->carState = state;
        this->carState->setOccupancy(lart_common::CAR_OCCUPANCY);
    }

    void Environment::addCone(const lart_common::Cone& cone) {
        std::shared_ptr<lart_common::State> coneState = std::make_shared<lart_common::State>();
        coneState->setPosition(cone);
        switch (cone.getColor()) {
            case lart_common::BLUE:
                coneState->setOccupancy(lart_common::BLUE_CONE_OCCUPANCY);
                break;
            case lart_common::YELLOW:
                coneState->setOccupancy(lart_common::YELLOW_CONE_OCCUPANCY);
                break;
            case lart_common::ORANGE:
                coneState->setOccupancy(lart_common::ORANGE_CONE_OCCUPANCY);
                break;
            case lart_common::UNKNOWN:
                coneState->setOccupancy(lart_common::UNKNOWN_CONE_OCCUPANCY);
                break;
        }
        this->cones.insert(coneState);
    }

    std::shared_ptr<lart_common::State> Environment::generateGraph() {

        if(this->carState == nullptr)
            throw std::runtime_error("Car state was not defined!");

        if(this->goalState == nullptr)
            throw std::runtime_error("Goal state was not defined!");

        std::unordered_map<lart_common::Point,std::shared_ptr<lart_common::State>> statesByPosition;

        Delaunay dt;
        Delaunay dtWithMidpoints;

        // insert all cones into the triangulation
        for(auto iter : this->cones) {
            statesByPosition[iter->getPosition()] = iter;
            dt.insert(iter->getPosition().getAsCGALPoint());
            dtWithMidpoints.insert(iter->getPosition().getAsCGALPoint());
        }

        auto initial_edges = dt.finite_edges();

        // re-triangulate with the midpoints
        for(auto iter = dt.finite_edges_begin(); iter != dt.finite_edges_end(); ++iter) {
            // get the vertices of this edge
            K::Point_2 p1 = iter->first->vertex((iter->second + 1) % 3)->point();
            K::Point_2 p2 = iter->first->vertex((iter->second + 2) % 3)->point();

            double newX = (p1.x() + p2.x()) / 2;
            double newY = (p1.y() + p2.y()) / 2;

            lart_common::Point newStatePosition = lart_common::Point(newX, newY);
            std::shared_ptr<lart_common::State> newState = std::make_shared<lart_common::State>(newStatePosition);
            statesByPosition[newStatePosition] = newState;

            // add the midpoint to the triangulation
            dtWithMidpoints.insert(K::Point_2(newX, newY));
        }

        // ONLY ADD CAR AND GOAL ON THE FINAL TRIANGULATION
        // insert the car position into the triangulation
        dt.insert(this->carState->getPosition().getAsCGALPoint());
        dtWithMidpoints.insert(this->carState->getPosition().getAsCGALPoint());
        statesByPosition[this->carState->getPosition()] = this->carState;

        // insert the goal position
        dt.insert(this->goalState->getPosition().getAsCGALPoint());
        dtWithMidpoints.insert(this->goalState->getPosition().getAsCGALPoint());
        statesByPosition[this->goalState->getPosition()] = this->goalState;

        // create the relations
        for(auto iter = dtWithMidpoints.finite_edges_begin(); iter != dtWithMidpoints.finite_edges_end(); ++iter) {
            // get the vertices of this edge
            K::Point_2 p1 = iter->first->vertex((iter->second + 1) % 3)->point();
            K::Point_2 p2 = iter->first->vertex((iter->second + 2) % 3)->point();

            // build points just to perform map lookup
            lart_common::Point p1Query = lart_common::Point(p1.x(), p1.y());
            lart_common::Point p2Query = lart_common::Point(p2.x(), p2.y());

            // add p2 as neighbor of p1
            statesByPosition[p1Query]->addNeighbor(statesByPosition[p2Query]);
            // add p1 as neightbor of p2
            statesByPosition[p2Query]->addNeighbor(statesByPosition[p1Query]);

        }

        // return the current car state as an entry point
        return this->carState;
    }

    std::shared_ptr<lart_common::State> Environment::getGoalState() const {
        return this->goalState;
    }

    void Environment::setGoalState(const std::shared_ptr<lart_common::State> &state) {
        this->goalState = state;
    }

    void Environment::computeGoalInFront(double distance) {

        if(this->carState == nullptr)
            throw std::runtime_error("Car state not defined, can't compute goal position!");

        // allocate the goal state
        std::shared_ptr<lart_common::State> goalState = std::make_shared<lart_common::State>();

        // set the position "distance" metres in front of the car
        lart_common::Point goalPosition = lart_common::Point(this->carState->getPosition().getX() +
                (distance * sin(this->carState->getPosition().getTheta())),
                                                               (this->carState->getPosition().getY() +
                                                                       (distance * cos(this->carState->getPosition().getTheta()))));
        goalState->setPosition(goalPosition);

        // set the goal state pointer
        this->goalState = goalState;
        goalState.reset();
    }

} // path_planner