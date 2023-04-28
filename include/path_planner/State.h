//
// Created by carlostojal on 27/04/2023.
//

#ifndef DELAUNAY_PATH_PLANNER_CORE_STATE_H
#define DELAUNAY_PATH_PLANNER_CORE_STATE_H

#include <path_planner/Point.h>
#include <set>
#include <memory>

namespace path_planner {

    enum occupancy_type_t {
        CAR_OCCUPANCY,
        YELLOW_CONE_OCCUPANCY,
        BLUE_CONE_OCCUPANCY,
        ORANGE_CONE_OCCUPANCY,
        FREE_SPACE
    };

    class State {
        private:
            path_planner::Point position;
            std::set<std::shared_ptr<State>> neighbors;
            occupancy_type_t occupancy = FREE_SPACE;

        public:
            State();
            State(State const &other);
            State(Point const &point);

            path_planner::Point getPosition() const;
            void setPosition(path_planner::Point position);

            void addNeighbor(std::shared_ptr<path_planner::State> neighbor);
            std::set<std::shared_ptr<State>> getNeighbors() const;

            occupancy_type_t getOccupancy() const;
            void setOccupancy(occupancy_type_t occupancy);

            // create a state in the mean distance between the current and the neighbor
            std::shared_ptr<path_planner::State> createIntermediate(std::shared_ptr<path_planner::State> neighbor);
    };

} // path_planner

#endif //DELAUNAY_PATH_PLANNER_CORE_STATE_H
