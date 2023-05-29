//
// Created by carlostojal on 05-05-2023.
//

#define ESTIMATED_SENSOR_RANGE 5 // estimated sensor range in meters

#include <delaunay_path_planner_core/PathPlanner.h>

namespace path_planner {

    path_planner::Path PathPlanner::search(const Environment &e) {
        path_planner::Path path;

        // the planning starting state is the current car state
        std::shared_ptr<path_planner::State> currentState = e.getCarState();
        if(currentState == nullptr)
            throw std::runtime_error("Tried to search on an environment with unknown car state!");

        std::shared_ptr<path_planner::State> goalState = e.getGoalState();
        if(goalState == nullptr)
            throw std::runtime_error("Tried to search on an environment without a defined goal state!");

        // the state comparator here is pretty standard
        auto comparator = [](const std::shared_ptr<path_planner::State>& s1,
                const std::shared_ptr<path_planner::State>& s2) {
            return s1 == s2;
        };

        // find the path
        path_planner::search::AStar astar = path_planner::search::AStar<path_planner::search::heuristics::PathFindingHeuristic>();
        astar.setInitialState(currentState);
        astar.setGoalState(goalState);

        // TODO: causes bad_function_call
        path = astar.search();

        // free the starting state pointer
        currentState.reset();
        // free the goal state pointer
        goalState.reset();

        return path;
    }
} // path_planner