//
// Created by carlostojal on 28/04/2023.
//

#ifndef DELAUNAY_PATH_PLANNER_CORE_SEARCHALGORITHM_H
#define DELAUNAY_PATH_PLANNER_CORE_SEARCHALGORITHM_H

#include <list>
#include <memory>
#include <delaunay_path_planner_core/State.h>
#include <delaunay_path_planner_core/search/heuristics/Heuristic.h>
#include <delaunay_path_planner_core/Path.h>

namespace path_planner::search {

    /*! \brief Search algorithm class. Represents a generic search algorithm, be it informed or uninformed. */
    class SearchAlgorithm {
        protected:
            /*! \brief The initial state of the search problem. */
            std::shared_ptr<path_planner::State> initialState = nullptr;

            /*! \brief The goal state of the search problem. */
            std::shared_ptr<path_planner::State> goalState = nullptr;

            /*! \brief Hard limit of iterations on the search. May be unused by some implementations. */
            size_t maxIterations = 99;

            /*! \brief Comparison function used to compare states, for example for goal detection. */
            std::function<bool(const std::shared_ptr<path_planner::State>&,
                    const std::shared_ptr<path_planner::State>&)> cmp = nullptr;

        public:
            /*! \brief Set the problem initial state.
             *
             * @param initial A shared pointer to the initial state.
             * */
            void setInitialState(const std::shared_ptr<path_planner::State>& initial);

            /*! \brief Set the problem goal state.
             *
             * @param goal A shared pointer to the goal state.
             * */
            void setGoalState(const std::shared_ptr<path_planner::State>& goal);

            /*! \brief Get the set maximum number of iterations.
             *
             * @return The maximum number of iterations.
             * */
            size_t getMaxIterations() const;

            /*! \brief Set the maximum number of iterations
             *
             * @param n_iterations The maximum number of iterations to run.
             * */
            void setMaxIterations(size_t n_iterations);

            /*! \brief Set the comparison function.
             *
             * @param cmp Comparison function. Receives shared pointers to 2 states to compare. Returns true if equal.
             * */
            void setComparator(std::function<bool(const std::shared_ptr<path_planner::State>&,
                    const std::shared_ptr<path_planner::State>&)> cmp);

            /*! \brief Pure virtual function which implements the search algorithm.
             *
             * @return The path from the initial state to the goal state of the instance.
             * */
            virtual path_planner::Path search() = 0;

            path_planner::Path constructPath(const std::unordered_map<std::shared_ptr<path_planner::State>,
                    std::shared_ptr<path_planner::State>>& parent,
                    std::shared_ptr<path_planner::State> currentState);
    };

} // path_planner

#endif //DELAUNAY_PATH_PLANNER_CORE_SEARCHALGORITHM_H
