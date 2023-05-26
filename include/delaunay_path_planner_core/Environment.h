//
// Created by carlostojal on 27/04/2023.
//

#ifndef DELAUNAY_PATH_PLANNER_CORE_ENVIRONMENT_H
#define DELAUNAY_PATH_PLANNER_CORE_ENVIRONMENT_H

#include <set>
#include <vector>
#include <unordered_map>
#include <memory>
#include <delaunay_path_planner_core/Cone.h>
#include <delaunay_path_planner_core/State.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Delaunay_triangulation_2<K> Delaunay;

namespace path_planner {

    /*!
     * @brief A class representing the environment the car is operating on.
     * The operating environment is characterized as a collection of cones and the state of the car.
     */
    class Environment {
        private:
            /*! @brief The collection of cones on the environment. */
            std::set<std::shared_ptr<path_planner::State>> cones;
            /*! @brief The current state of the car on the environnment. */
            std::shared_ptr<path_planner::State> carState = nullptr;
            std::shared_ptr<path_planner::State> goalState = nullptr;

        public:
            Environment();
            ~Environment();

            /*! @brief Get the cone collection. */
            std::set<std::shared_ptr<path_planner::State>> getCones() const;

            /*! @brief Get the car state. */
            std::shared_ptr<path_planner::State> getCarState() const;

            /*! @brief Set/update the car state.
             *
             * @param state State to set.
             */
            void setCarState(const std::shared_ptr<path_planner::State>& state);
            /*! \brief Get the car's goal state. */
            std::shared_ptr<path_planner::State> getGoalState() const;
            /*! \brief Set the ca's goal state. */
            void setGoalState(const std::shared_ptr<path_planner::State>& state);
            /*! @brief Add a cone to the environment.
             *
             * @param cone Cone to add.
             */
            void addCone(const path_planner::Cone& cone);

            /*! @brief Generate the graph by doing a double Delaunay triangulation.
             * The states get a relation among them.
             *
             * @return The starting node of the graph (the car state)
             */
            std::shared_ptr<State> generateGraph();

            /*! \brief Set the goal state as a point "distance" metres in front of the car.
             *
             * @param distance The distance from the car to the goal to generate.
             * */
            void computeGoalInFront(double distance);
    };

} // path_planner

#endif //DELAUNAY_PATH_PLANNER_CORE_ENVIRONMENT_H
