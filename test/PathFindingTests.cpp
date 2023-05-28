//
// Created by carlostojal on 27-05-2023.
//

#include <gtest/gtest.h>
#include <delaunay_path_planner_core/Environment.h>
#include <delaunay_path_planner_core/State.h>
#include <delaunay_path_planner_core/PathPlanner.h>
#include <delaunay_path_planner_core/tests/Misc.h>

class PathFindingTests : public ::testing::Test {

protected:

    path_planner::Environment environment;
    std::shared_ptr<path_planner::State> initialState = nullptr;
    path_planner::PathPlanner planner;

    virtual void SetUp() {
        environment = path_planner::tests::Misc::generateStraightSegment();
        initialState = environment.generateGraph();
    }

    virtual void TearDown() {

    }

};

TEST_F(PathFindingTests, straightTrackSegmentPlanning) {

    path_planner::Path p = path_planner::PathPlanner::search(this->environment);

    // TODO: compare the path
    path_planner::Path expectedPath; // TODO

    ASSERT_EQ(expectedPath, p);
}

TEST_F(PathFindingTests, curvedTrackSegmentPlanning) {

    // TODO: test on a curve
    FAIL();
}

TEST_F(PathFindingTests, inconsistentTrackSegmentPlanning) {

    // TODO: test on a segment with mis-colored cones
    FAIL();
}