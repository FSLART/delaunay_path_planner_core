//
// Created by carlostojal on 27/04/2023.
//

#ifndef DELAUNAY_PATH_PLANNER_CORE_STATE_H
#define DELAUNAY_PATH_PLANNER_CORE_STATE_H

#include <path_planner/Point.h>
#include <set>
#include <memory>

namespace path_planner {

    class State {
        private:
            path_planner::Point position;
            std::set<std::shared_ptr<State>> neighbors;

        public:
            State();
            State(State const &other);
            State(Point const &point);

            path_planner::Point getPosition() const;
            void setPosition(path_planner::Point position);
            void addNeighbor(std::shared_ptr<path_planner::State> neighbor);
            std::set<std::shared_ptr<State>> getNeighbors() const;

            // create a state in the mean distance between the current and the neighbor
            void createIntermediate(std::shared_ptr<path_planner::State> neighbor);
    };

} // path_planner

#endif //DELAUNAY_PATH_PLANNER_CORE_STATE_H
