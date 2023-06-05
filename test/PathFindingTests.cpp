//
// Created by carlostojal on 27-05-2023.
//

#include <gtest/gtest.h>
#include <delaunay_path_planner_core/Environment.h>
#include <lart_common/State.h>
#include <delaunay_path_planner_core/PathPlanner.h>
#include <delaunay_path_planner_core/tests/Misc.h>

class PathFindingTests : public ::testing::Test {

protected:

    path_planner::Environment environment;
    std::shared_ptr<lart_common::State> initialState = nullptr;

    path_planner::Environment inconsistentEnvironment;
    std::shared_ptr<lart_common::State> inconsistentInitialState = nullptr;

    path_planner::Environment conesOnSideEnvironment;
    std::shared_ptr<lart_common::State> conesOnSideInitialState = nullptr;

    path_planner::PathPlanner planner;

    virtual void SetUp() {
        environment = path_planner::tests::Misc::generateStraightSegment();
        initialState = environment.generateGraph();

        inconsistentEnvironment = path_planner::tests::Misc::generateInconsistentSegment();
        inconsistentInitialState = inconsistentEnvironment.generateGraph();

        conesOnSideEnvironment = path_planner::tests::Misc::generateSegmentWithConesOnSide();
        conesOnSideInitialState = conesOnSideEnvironment.generateGraph();

    }

    virtual void TearDown() {

    }

};

TEST_F(PathFindingTests, straightTrackSegmentPlanning) {

    lart_common::Path p = path_planner::PathPlanner::search(this->environment);

    lart_common::Path expectedPath;

    std::shared_ptr<lart_common::State> s1 = std::make_shared<lart_common::State>(lart_common::Point(0, 0));
    std::shared_ptr<lart_common::State> s2 = std::make_shared<lart_common::State>(lart_common::Point(0, 1));
    std::shared_ptr<lart_common::State> s3 = std::make_shared<lart_common::State>(lart_common::Point(0, 2));
    std::shared_ptr<lart_common::State> s4 = std::make_shared<lart_common::State>(lart_common::Point(0, 3));
    std::shared_ptr<lart_common::State> s5 = std::make_shared<lart_common::State>(lart_common::Point(0, 4));
    std::shared_ptr<lart_common::State> s6 = std::make_shared<lart_common::State>(lart_common::Point(0, 5));

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

    s1.reset();
    s2.reset();
    s3.reset();
    s4.reset();
    s5.reset();
    s6.reset();
}

/*
TEST_F(PathFindingTests, curvedTrackSegmentPlanning) {

    // TODO: test on a curve
    FAIL();
}*/

TEST_F(PathFindingTests, inconsistentTrackSegmentPlanning) {

    lart_common::Path p = path_planner::PathPlanner::search(this->inconsistentEnvironment);

    lart_common::Path expectedPath;

    std::shared_ptr<lart_common::State> s1 = std::make_shared<lart_common::State>(lart_common::Point(0, 0));
    std::shared_ptr<lart_common::State> s2 = std::make_shared<lart_common::State>(lart_common::Point(0, 1));
    std::shared_ptr<lart_common::State> s3 = std::make_shared<lart_common::State>(lart_common::Point(0, 2));
    std::shared_ptr<lart_common::State> s4 = std::make_shared<lart_common::State>(lart_common::Point(0, 3));
    std::shared_ptr<lart_common::State> s5 = std::make_shared<lart_common::State>(lart_common::Point(0, 4));
    std::shared_ptr<lart_common::State> s6 = std::make_shared<lart_common::State>(lart_common::Point(0, 5));

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

    s1.reset();
    s2.reset();
    s3.reset();
    s4.reset();
    s5.reset();
    s6.reset();
}

TEST_F(PathFindingTests, trackWithConesOnSidePlanning) {

    lart_common::Path p = path_planner::PathPlanner::search(this->conesOnSideEnvironment);

    lart_common::Path expectedPath;

    std::shared_ptr<lart_common::State> s1 = std::make_shared<lart_common::State>(lart_common::Point(0, 0));
    std::shared_ptr<lart_common::State> s2 = std::make_shared<lart_common::State>(lart_common::Point(0, 1));
    std::shared_ptr<lart_common::State> s3 = std::make_shared<lart_common::State>(lart_common::Point(0, 2));
    std::shared_ptr<lart_common::State> s4 = std::make_shared<lart_common::State>(lart_common::Point(0, 3));
    std::shared_ptr<lart_common::State> s5 = std::make_shared<lart_common::State>(lart_common::Point(0, 4));
    std::shared_ptr<lart_common::State> s6 = std::make_shared<lart_common::State>(lart_common::Point(0, 5));

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

    s1.reset();
    s2.reset();
    s3.reset();
    s4.reset();
    s5.reset();
    s6.reset();
}