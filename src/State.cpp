//
// Created by carlostojal on 27/04/2023.
//

#include "../include/path_planner/State.h"

namespace path_planner {

    State::State() {
        this->position = path_planner::Point();
    }

    State::State(const State &other) {
        this->position = other.position;
    }

    State::State(const Point &point) {
        this->position = point;
    }

    path_planner::Point State::getPosition() const {
        return this->position;
    }

    void State::setPosition(path_planner::Point position) {
        this->position = position;
    }

    bool State::hasNeighbor(std::shared_ptr<path_planner::State> neighbor) {
        return this->neighbors.count(neighbor) > 0;
    }

    void State::addNeighbor(std::shared_ptr<path_planner::State> neighbor) {
        if(neighbor == nullptr)
            throw std::runtime_error("Tried to add a null neighbor!");

        this->neighbors.insert(neighbor);
    }

    void State::removeNeighbor(std::shared_ptr<path_planner::State> neighbor) {
        if(!this->hasNeighbor(neighbor))
            throw std::runtime_error("Tried to remove a state which is not a neighbor!");

        this->neighbors.erase(neighbor);
    }

    std::set<std::shared_ptr<State>> State::getNeighbors() const {
        return this->neighbors;
    }

    std::shared_ptr<path_planner::State> State::createIntermediate(std::shared_ptr<path_planner::State> neighbor) {
        // verify the state is a neighbor
        if(!this->hasNeighbor(neighbor))
            throw std::runtime_error("State is not a neighbor!");

        // compute the new point
        path_planner::Point newPoint((this->position.getX() + neighbor->position.getX()) / 2,
                                     (this->position.getY() + neighbor->position.getY()) / 2);

        // create the new state
        std::shared_ptr<path_planner::State> newState = std::make_shared<path_planner::State>(newPoint);

        this->neighbors.insert(newState);

        return newState;
    }

    occupancy_type_t State::getOccupancy() const {
        return this->occupancy;
    }

    void State::setOccupancy(occupancy_type_t occupancy) {
        this->occupancy = occupancy;
    }

    bool State::operator==(const State& other) const {
        return this->position == other.position;
    }

} // path_planner