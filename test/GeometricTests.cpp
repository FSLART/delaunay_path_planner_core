//
// Created by carlostojal on 27-05-2023.
//

#include <gtest/gtest.h>
#include <delaunay_path_planner_core/Point.h>

class GeometricTests : public ::testing::Test {

    protected:

        path_planner::Point p1;
        path_planner::Point p2;

        path_planner::Point p3;

        virtual void SetUp() {
            // considered the same. 2 decimal places considered to comparison
            p1 = path_planner::Point(1.253, 5);
            p2 = path_planner::Point(1.252, 5);

            p3 = path_planner::Point(2.5, 9.312);
        }

        virtual void TearDown() {

        }

};

TEST_F(GeometricTests, comparePointsWithDifferentPosition) {

    ASSERT_NE(p1, p3);
}

TEST_F(GeometricTests, comparePointsWithSamePosition) {

    ASSERT_EQ(p1, p2);
}

TEST_F(GeometricTests, computeDistanceBetweenPoints) {

    double dist = p1.distanceTo(p3);

    ASSERT_EQ((int) (dist * 100), 448);
}