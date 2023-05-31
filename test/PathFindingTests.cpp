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

    path_planner::Environment inconsistentEnvironment;
    std::shared_ptr<path_planner::State> inconsistentInitialState = nullptr;

    path_planner::PathPlanner planner;

    virtual void SetUp() {
        environment = path_planner::tests::Misc::generateStraightSegment();
        initialState = environment.generateGraph();

        inconsistentEnvironment = path_planner::tests::Misc::generateInconsistentSegment();
        inconsistentInitialState = inconsistentEnvironment.generateGraph();

    }

    virtual void TearDown() {

    }

};

TEST_F(PathFindingTests, comparePaths) {

    path_planner::Path p1 = path_planner::Path();
    path_planner::Path p2 = path_planner::Path();

    std::shared_ptr<path_planner::State> s1 = std::make_shared<path_planner::State>(path_planner::Point(1.0f, 2.0f));
    std::shared_ptr<path_planner::State> s2 = std::make_shared<path_planner::State>(path_planner::Point(5.2f, 7.1f));
    std::shared_ptr<path_planner::State> s3 = std::make_shared<path_planner::State>(path_planner::Point(3.1f, 4.5f));

    s1->addNeighbor(s2);
    s2->addNeighbor(s3);

    p1.addState(s1);
    p2.addState(s1);

    p1.addState(s2);
    p2.addState(s2);

    p1.addState(s3);
    p2.addState(s3);

    std::cout << "Path 1: " << p1._str_() << std::endl;
    std::cout << "Path 2: " << p2._str_() << std::endl;

    ASSERT_EQ(p1, p2);

    s1.reset();
    s2.reset();
    s3.reset();
}

TEST_F(PathFindingTests, straightTrackSegmentPlanning) {

    path_planner::Path p = path_planner::PathPlanner::search(this->environment);

    path_planner::Path expectedPath;

    std::shared_ptr<path_planner::State> s1 = std::make_shared<path_planner::State>(path_planner::Point(0, 0));
    std::shared_ptr<path_planner::State> s2 = std::make_shared<path_planner::State>(path_planner::Point(0, 1));
    std::shared_ptr<path_planner::State> s3 = std::make_shared<path_planner::State>(path_planner::Point(0, 2));
    std::shared_ptr<path_planner::State> s4 = std::make_shared<path_planner::State>(path_planner::Point(0, 3));
    std::shared_ptr<path_planner::State> s5 = std::make_shared<path_planner::State>(path_planner::Point(0, 4));
    std::shared_ptr<path_planner::State> s6 = std::make_shared<path_planner::State>(path_planner::Point(0, 5));

    s1->addNeighbor(s2);
    s2->addNeighbor(s3);
    s3->addNeighbor(s4);
    s4->addNeighbor(s5);
    s5->addNeighbor(s6);

    expectedPath.addState(s1);
    expectedPath.addState(s2);
    expectedPath.addState(s3);
    expectedPath.addState(s4);
    expectedPath.addState(s5);
    expectedPath.addState(s6);

    std::cout << "This path was found: " << p._str_() << std::endl;
    std::cout << "This path was expected: " << expectedPath._str_() << std::endl;

    ASSERT_TRUE(p == expectedPath);
}

/*
TEST_F(PathFindingTests, curvedTrackSegmentPlanning) {

    // TODO: test on a curve
    FAIL();
}*/

TEST_F(PathFindingTests, inconsistentTrackSegmentPlanning) {

    path_planner::Path p = path_planner::PathPlanner::search(this->inconsistentEnvironment);

    path_planner::Path expectedPath;

    std::shared_ptr<path_planner::State> s1 = std::make_shared<path_planner::State>(path_planner::Point(0, 0));
    std::shared_ptr<path_planner::State> s2 = std::make_shared<path_planner::State>(path_planner::Point(0, 1));
    std::shared_ptr<path_planner::State> s3 = std::make_shared<path_planner::State>(path_planner::Point(0, 2));
    std::shared_ptr<path_planner::State> s4 = std::make_shared<path_planner::State>(path_planner::Point(0, 3));
    std::shared_ptr<path_planner::State> s5 = std::make_shared<path_planner::State>(path_planner::Point(0, 4));
    std::shared_ptr<path_planner::State> s6 = std::make_shared<path_planner::State>(path_planner::Point(0, 5));

    s1->addNeighbor(s2);
    s2->addNeighbor(s3);
    s3->addNeighbor(s4);
    s4->addNeighbor(s5);
    s5->addNeighbor(s6);

    expectedPath.addState(s1);
    expectedPath.addState(s2);
    expectedPath.addState(s3);
    expectedPath.addState(s4);
    expectedPath.addState(s5);
    expectedPath.addState(s6);

    std::cout << "This path was found: " << p._str_() << std::endl;
    std::cout << "This path was expected: " << expectedPath._str_() << std::endl;

    ASSERT_TRUE(p == expectedPath);
}