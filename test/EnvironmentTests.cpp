//
// Created by carlostojal on 26-05-2023.
//

#include <gtest/gtest.h>
#include "delaunay_path_planner_core/tests/Misc.h"

#define GOAL_DISTANCE 5

class EnvironmentTests : public ::testing::Test {

    protected:
        path_planner::Environment straightSegmentEnvironment;

        virtual void SetUp() {
            straightSegmentEnvironment = path_planner::tests::Misc::generateStraightSegment();
            straightSegmentEnvironment.computeGoalInFront(5.0);
        }

        virtual void TearDown() {

        }
};

TEST_F(EnvironmentTests, createStraightSegmentGraph) {

    std::shared_ptr<path_planner::State> startingState = straightSegmentEnvironment.generateGraph();

    // in the end, if the starting state has no neighbors something went wrong
    ASSERT_GT(straightSegmentEnvironment.getCarState()->getNeighbors().size(), 0);
}