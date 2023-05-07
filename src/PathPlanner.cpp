//
// Created by carlostojal on 05-05-2023.
//

#include <delaunay_path_planner_core/PathPlanner.h>

#define MAX_SEARCH_DEPTH 5

namespace path_planner {

    std::list<path_planner::Path> PathPlanner::search(const Environment &e) {
        std::list<path_planner::Path> paths;

        // the planning starting state is the current car state
        std::shared_ptr<path_planner::State> currentState = e.getCarState();
        if(currentState == nullptr)
            throw std::runtime_error("Tried to search on an environment with unknown car state!");

        /* TODO: the goal is an imaginary point in front of the car, as far as the estimated sensor range. Even though the car will not always be on straight lines, curves are not very meaningful in such a reduced horizon
         * TODO: use A* with the PathFindingHeuristic
         * */






        // free the starting state pointer
        currentState.reset();

        return paths;
    }
} // path_planner