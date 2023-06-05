//
// Created by carlostojal on 05-05-2023.
//

#define ESTIMATED_SENSOR_RANGE 5 // estimated sensor range in meters

#include <delaunay_path_planner_core/PathPlanner.h>

namespace path_planner {

    lart_common::Path PathPlanner::search(const Environment &e) {
        lart_common::Path path;

        // the planning starting state is the current car state
        std::shared_ptr<lart_common::State> currentState = e.getCarState();
        if(currentState == nullptr)
            throw std::runtime_error("Tried to search on an environment with unknown car state!");

        std::shared_ptr<lart_common::State> goalState = e.getGoalState();
        if(goalState == nullptr)
            throw std::runtime_error("Tried to search on an environment without a defined goal state!");

        // the state comparator here is pretty standard
        auto comparator = [](const std::shared_ptr<lart_common::State>& s1,
                const std::shared_ptr<lart_common::State>& s2) {
            return s1 == s2;
        };

        // find the path
        path_planner::search::AStar astar = path_planner::search::AStar<path_planner::search::heuristics::PathFindingHeuristic>();
        astar.setInitialState(currentState);
        astar.setGoalState(goalState);
        astar.setComparator(comparator);

        path = astar.search();

        // free the starting state pointer
        currentState.reset();
        // free the goal state pointer
        goalState.reset();

        return path;
    }
} // path_planner