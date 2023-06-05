//
// Created by carlostojal on 26-05-2023.
//
#include "delaunay_path_planner_core/tests/Misc.h"

#define GOAL_DISTANCE 5.0

path_planner::Environment path_planner::tests::Misc::generateStraightSegment() {
    // create environment instance
    path_planner::Environment env = path_planner::Environment();

    // initialize car state
    std::shared_ptr<lart_common::State> carState = std::make_shared<lart_common::State>();
    carState->setPosition(lart_common::Point(0, 0, 0));

    env.setCarState(carState);
    env.computeGoalInFront(GOAL_DISTANCE);

    // add the cones
    env.addCone(lart_common::Cone(-2, 1, lart_common::YELLOW));
    env.addCone(lart_common::Cone(2, 1, lart_common::BLUE));
    env.addCone(lart_common::Cone(-2, 3, lart_common::YELLOW));
    env.addCone(lart_common::Cone(2, 3, lart_common::BLUE));
    env.addCone(lart_common::Cone(-2, 5, lart_common::YELLOW));
    env.addCone(lart_common::Cone(2, 5, lart_common::BLUE));

    return env;
}

path_planner::Environment path_planner::tests::Misc::generateInconsistentSegment() {
    // create environment instance
    path_planner::Environment env = path_planner::Environment();

    // initialize car state
    std::shared_ptr<lart_common::State> carState = std::make_shared<lart_common::State>();
    carState->setPosition(lart_common::Point(0, 0, 0));

    env.setCarState(carState);
    env.computeGoalInFront(GOAL_DISTANCE);

    // add the cones
    env.addCone(lart_common::Cone(-2, 1, lart_common::YELLOW));
    env.addCone(lart_common::Cone(2, 1, lart_common::YELLOW)); // this cone is wrong
    env.addCone(lart_common::Cone(-2, 3, lart_common::BLUE));
    env.addCone(lart_common::Cone(2, 3, lart_common::BLUE)); // this cone is wrong
    env.addCone(lart_common::Cone(-2, 5, lart_common::YELLOW));
    env.addCone(lart_common::Cone(2, 5, lart_common::BLUE));

    return env;
}

path_planner::Environment path_planner::tests::Misc::generateCurvedSegment() {
    // TODO
    return path_planner::Environment();
}

path_planner::Environment path_planner::tests::Misc::generateSegmentWithConesOnSide() {
    // create environment instance
    path_planner::Environment env = path_planner::Environment();

    // initialize car state
    std::shared_ptr<lart_common::State> carState = std::make_shared<lart_common::State>();
    carState->setPosition(lart_common::Point(0, 0, 0));

    env.setCarState(carState);
    env.computeGoalInFront(GOAL_DISTANCE);

    // add the cones
    env.addCone(lart_common::Cone(-2, 1, lart_common::YELLOW));
    env.addCone(lart_common::Cone(2, 1, lart_common::BLUE));
    env.addCone(lart_common::Cone(-2, 3, lart_common::YELLOW));
    env.addCone(lart_common::Cone(2, 3, lart_common::BLUE));
    env.addCone(lart_common::Cone(-2, 5, lart_common::YELLOW));
    env.addCone(lart_common::Cone(2, 5, lart_common::BLUE));

    // add some cones on the sides
    env.addCone(lart_common::Cone(3, 3));
    env.addCone(lart_common::Cone(-4, 5));

    return env;
}
