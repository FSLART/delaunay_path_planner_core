//
// Created by carlostojal on 27/04/2023.
//

#ifndef DELAUNAY_PATH_PLANNER_CORE_STATE_H
#define DELAUNAY_PATH_PLANNER_CORE_STATE_H

#include <delaunay_path_planner_core/Point.h>
#include <set>
#include <memory>

namespace path_planner {

    enum occupancy_type_t {
        CAR_OCCUPANCY,
        YELLOW_CONE_OCCUPANCY,
        BLUE_CONE_OCCUPANCY,
        ORANGE_CONE_OCCUPANCY,
        FREE_SPACE
    };

    /*! @brief This class represents a state in the planning environment */
    class State {
        private:
            /*! @brief The position of the state in the world */
            path_planner::Point position;
            /*! @brief The neighbors (states the car is able to transition to) from this state */
            std::set<std::shared_ptr<State>> neighbors;
             /*! @brief Characterization of the state in terms of occupancy */
            occupancy_type_t occupancy = FREE_SPACE;

        public:
            State();
            /*! @brief Instantiate a State by copying the values of another */
            State(State const &other);
            /*! @brief Instantiate a State with a position */
            State(Point const &point);

            /*! @brief Get the position of the state in the world */
            path_planner::Point getPosition() const;
            /*! @brief Set the position of the state in the world */
            void setPosition(path_planner::Point position);

            /*! @brief Check if a state is present in the neighbor list of this instance */
            bool hasNeighbor(std::shared_ptr<path_planner::State> neighbor);
            /*! @brief Add a neighbor state to this instance */
            void addNeighbor(std::shared_ptr<path_planner::State> neighbor);
            /*! @brief Remove a state from the neighbor list of this instance */
            void removeNeighbor(std::shared_ptr<path_planner::State> neighbor);
            /*! @brief Get the neighbor list of this instance */
            std::set<std::shared_ptr<State>> getNeighbors() const;

            /*! @brief Get the characterization of the state in terms of occupancy */
            occupancy_type_t getOccupancy() const;
            /*! @brief Set the characterization of the occupancy */
            void setOccupancy(occupancy_type_t occupancy);

            // create a state in the mean distance between the current and the neighbor
            /*! @brief Create a state in the mean distance between this instance and the provided neighbor
             *
             * @param neighbor The state to add as neighbor
             * @return The created state (mean distance)
             */
            std::shared_ptr<path_planner::State> createIntermediate(std::shared_ptr<path_planner::State> neighbor);

            /*! @brief Compare this state with another */
            bool operator==(const State& other) const;
    };

} // path_planner

#endif //DELAUNAY_PATH_PLANNER_CORE_STATE_H
