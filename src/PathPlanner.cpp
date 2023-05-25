//
// Created by carlostojal on 05-05-2023.
//

#define ESTIMATED_SENSOR_RANGE 10 // estimated sensor range in meters

#include <delaunay_path_planner_core/PathPlanner.h>

namespace path_planner {

    std::list<path_planner::Path> PathPlanner::search(const Environment &e) {
        std::list<path_planner::Path> paths;

        // the planning starting state is the current car state
        std::shared_ptr<path_planner::State> currentState = e.getCarState();
        if(currentState == nullptr)
            throw std::runtime_error("Tried to search on an environment with unknown car state!");

        // create a goal state
        std::shared_ptr<path_planner::State> goalState = std::make_shared<path_planner::State>();
        double goalX = e.getCarState()->getPosition().getX() + cos(e.getCarState()->getPosition().getTheta()) * ESTIMATED_SENSOR_RANGE;
        double goalY = e.getCarState()->getPosition().getY() + sin(e.getCarState()->getPosition().getTheta()) * ESTIMATED_SENSOR_RANGE;
        goalState->setPosition(path_planner::Point(goalX, goalY));

        // find the path
        path_planner::search::AStar astar = path_planner::search::AStar<path_planner::search::heuristics::PathFindingHeuristic>();
        astar.setInitialState(e.getCarState());
        astar.setGoalState(goalState);

        // free the starting state pointer
        currentState.reset();
        // free the goal state pointer
        goalState.reset();

        return paths;
    }
} // path_planner