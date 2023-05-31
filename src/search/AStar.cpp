//
// Created by carlostojal on 28/04/2023.
//

#include <delaunay_path_planner_core/search/AStar.h>
#include "delaunay_path_planner_core/search/heuristics/PathFindingHeuristic.h"

#define MAX_SEARCH_ITERATIONS 200

namespace path_planner::search {

    template<typename HeuristicT>
    AStar<HeuristicT>::AStar() = default;

    template <typename HeuristicT>
    path_planner::Path AStar<HeuristicT>::search() {

        if(this->cmp == nullptr)
            throw std::runtime_error("State comparison criteria was not set!");

        path_planner::Path path;
        std::shared_ptr<path_planner::State> stateToExplore = this->initialState;
        std::priority_queue<std::pair<double,std::shared_ptr<path_planner::State>>> frontier;
        std::unordered_map<std::shared_ptr<path_planner::State>, std::shared_ptr<path_planner::State>> parent;
        std::unordered_map<std::shared_ptr<path_planner::State>, double> gScores;
        std::unordered_map<std::shared_ptr<path_planner::State>, double> fScores;

        frontier.push(std::make_pair(0.0, this->initialState));
        gScores[stateToExplore] = 0.0;
        fScores[stateToExplore] = this->initialState->getPosition().distanceTo(this->goalState->getPosition()); // f = 0 + h

        while (!frontier.empty()) {
            std::shared_ptr<path_planner::State> currentState = frontier.top().second;
            frontier.pop();

            if (this->cmp(currentState, this->goalState)) {
                // Goal state reached, construct the path
                path = this->constructPath(parent, currentState);
                return path;
            }

            for (const auto& child : currentState->getNeighbors()) {
                // this g score is the current node's g score plus the g cost of the child
                double tentativeGScore = gScores[currentState] + this->heuristic.compute(currentState, child,
                                                                                         this->goalState);

                if (gScores.find(child) == gScores.end() || tentativeGScore < gScores[child]) {
                    parent[child] = currentState;
                    gScores[child] = tentativeGScore;
                    fScores[child] = gScores[child] + this->heuristic.compute(currentState, child, this->goalState); // f-score based on g-score and heuristic

                    frontier.emplace(fScores[child], child);
                }
            }
        }

        return path;
    }

    template class AStar<path_planner::search::heuristics::PathFindingHeuristic>;
} // path_planner