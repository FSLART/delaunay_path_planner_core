//
// Created by carlostojal on 26-05-2023.
//

#include <gtest/gtest.h>
#include <delaunay_path_planner_core/tests/Misc.h>

#define GOAL_DISTANCE 5

class EnvironmentTests : public ::testing::Test {

    protected:
        path_planner::Environment straightSegmentEnvironment;

        virtual void SetUp() {
            straightSegmentEnvironment = path_planner::tests::Misc::generateStraightSegment();
            straightSegmentEnvironment.computeGoalInFront(GOAL_DISTANCE);
        }

        virtual void TearDown() {

        }
};

TEST_F(EnvironmentTests, createTriangleGraph) {

    // create starting state
    std::shared_ptr<path_planner::State> startingState =
            std::make_shared<path_planner::State>(path_planner::Point(0, 0));

    path_planner::Environment basicEnv;

    // add a cones to form a triangle with the starting state and goal state
    basicEnv.addCone(path_planner::Cone(-3, 3));

    // create goal state
    std::shared_ptr<path_planner::State> goalState =
            std::make_shared<path_planner::State>(path_planner::Point(3, 3));

    basicEnv.setCarState(startingState);
    basicEnv.setGoalState(goalState);

    basicEnv.generateGraph();

    // assert the starting state's neighbors are these 2 cones
    for(auto & n : startingState->getNeighbors())
        ASSERT_TRUE(n->getPosition() == path_planner::Point(-3, 3) || n->getPosition() == path_planner::Point(3, 3));

    startingState.reset();
    goalState.reset();
}

TEST_F(EnvironmentTests, createStraightSegmentGraph) {

    straightSegmentEnvironment = path_planner::tests::Misc::generateStraightSegment();
    straightSegmentEnvironment.computeGoalInFront(GOAL_DISTANCE);
    std::shared_ptr<path_planner::State> startingState = straightSegmentEnvironment.generateGraph();

    // in the end, if the starting state has no neighbors something went wrong
    ASSERT_GT(straightSegmentEnvironment.getCarState()->getNeighbors().size(), 0);

    startingState.reset();
}