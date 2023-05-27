//
// Created by carlostojal on 27-05-2023.
//

#include <gtest/gtest.h>
#include <delaunay_path_planner_core/Environment.h>

class PathFindingTests : public ::testing::Test {

protected:

    path_planner::Environment environment;

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

};

TEST_F(PathFindingTests, straightTrackSegmentPlanning) {

    // TODO: test on a straight segment
}

TEST_F(PathFindingTests, curvedTrackSegmentPlanning) {

    // TODO: test on a curve
}

TEST_F(PathFindingTests, inconsistentTrackSegmentPlanning) {

    // TODO: test on a segment with mis-colored cones
}