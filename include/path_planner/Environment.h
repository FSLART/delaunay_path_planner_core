//
// Created by carlostojal on 27/04/2023.
//

#ifndef DELAUNAY_PATH_PLANNER_CORE_ENVIRONMENT_H
#define DELAUNAY_PATH_PLANNER_CORE_ENVIRONMENT_H

#include <vector>
#include <memory>
#include <path_planner/Point.h>
#include <path_planner/State.h>

namespace path_planner {

    class Environment {
        private:
            std::vector<path_planner::Point> cones;
            std::shared_ptr<path_planner::State> carState = nullptr;

        public:
            Environment();

            std::vector<path_planner::Point> getCones() const;
            path_planner::State getCarState() const;

            void setCarState(const path_planner::State& state);
            void addCone(path_planner::Point cone);

            std::shared_ptr<State> generateGraph();
    };

} // path_planner

#endif //DELAUNAY_PATH_PLANNER_CORE_ENVIRONMENT_H
