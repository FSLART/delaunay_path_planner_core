//
// Created by carlostojal on 26-05-2023.
//

#include <gtest/gtest.h>
#include "delaunay_path_planner_core/tests/Misc.h"

class ConeSearchTests : public ::testing::Test {

    protected:
        path_planner::Environment straightSegmentEnvironment;

        virtual void SetUp() {
            straightSegmentEnvironment = path_planner::tests::Misc::generateStraightSegment();
        }

        virtual void TearDown() {

        }
};

TEST_F(ConeSearchTests, findClosestYellow) {

    // TODO
}

TEST_F(ConeSearchTests, findClosestBlue) {

    // TODO
}
