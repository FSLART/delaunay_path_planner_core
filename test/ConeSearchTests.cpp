//
// Created by carlostojal on 26-05-2023.
//

#include <gtest/gtest.h>
#include <delaunay_path_planner_core/tests/Misc.h>
#include <delaunay_path_planner_core/State.h>
#include <delaunay_path_planner_core/search/heuristics/PathFindingHeuristic.h>

class ConeSearchTests : public ::testing::Test {

    protected:
        path_planner::Environment straightSegmentEnvironment;

        virtual void SetUp() {
            straightSegmentEnvironment = path_planner::tests::Misc::generateStraightSegment();
            straightSegmentEnvironment.computeGoalInFront(5.0);
            straightSegmentEnvironment.generateGraph();
        }

        virtual void TearDown() {

        }
};

TEST_F(ConeSearchTests, findClosestYellow) {

    std::shared_ptr<path_planner::State> closestYellowCone = nullptr;
    path_planner::search::heuristics::PathFindingHeuristic::findClosestConeRoutine(path_planner::YELLOW_CONE_OCCUPANCY,
                           straightSegmentEnvironment.getCarState(),
                           closestYellowCone);

    path_planner::Point expectedPosition(-2, 1);

    if(closestYellowCone == nullptr)
        FAIL();

    ASSERT_EQ(expectedPosition, closestYellowCone->getPosition());
}

TEST_F(ConeSearchTests, findClosestBlue) {

    std::shared_ptr<path_planner::State> closestBlueCone = nullptr;
    path_planner::search::heuristics::PathFindingHeuristic::findClosestConeRoutine(path_planner::BLUE_CONE_OCCUPANCY,
                                                                                   straightSegmentEnvironment.getCarState(),
                                                                                   closestBlueCone);

    path_planner::Point expectedPosition(2, 1);

    if(closestBlueCone == nullptr)
        FAIL();

    ASSERT_EQ(expectedPosition, closestBlueCone->getPosition());
}
