//
// Created by carlostojal on 27/04/2023.
//

#ifndef DELAUNAY_PATH_PLANNER_CORE_ENVIRONMENT_H
#define DELAUNAY_PATH_PLANNER_CORE_ENVIRONMENT_H

#include <set>
#include <vector>
#include <memory>
#include <delaunay_path_planner_core/Cone.h>
#include <delaunay_path_planner_core/State.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Delaunay_triangulation_2<K> Delaunay;

namespace path_planner {

    class Environment {
        private:
            std::vector<path_planner::Cone> cones;
            std::shared_ptr<path_planner::State> carState = nullptr;

        public:
            Environment();

            std::vector<path_planner::Cone> getCones() const;
            path_planner::State getCarState() const;

            void setCarState(const path_planner::State& state);
            void addCone(const path_planner::Cone& cone);

            std::shared_ptr<State> generateGraph();
    };

} // path_planner

#endif //DELAUNAY_PATH_PLANNER_CORE_ENVIRONMENT_H
