//
// Created by carlostojal on 26-05-2023.
//
#include "delaunay_path_planner_core/tests/Misc.h"

path_planner::Environment path_planner::tests::Misc::generateStraightSegment() {
    // create environment instance
    path_planner::Environment env = path_planner::Environment();

    // initialize car state
    std::shared_ptr<path_planner::State> carState = std::make_shared<path_planner::State>();
    carState->setPosition(path_planner::Point(0, 0, 0));

    env.setCarState(carState);

    // add the cones
    env.addCone(path_planner::Cone(-2, 1));
    env.addCone(path_planner::Cone(2, 1));
    env.addCone(path_planner::Cone(-2, 3));
    env.addCone(path_planner::Cone(2, 3));
    env.addCone(path_planner::Cone(-2, 5));
    env.addCone(path_planner::Cone(2, 5));

    return env;
}

path_planner::Environment path_planner::tests::Misc::generateCurvedSegment() {
    // TODO
    return path_planner::Environment();
}
