//
// Created by carlostojal on 05-05-2023.
//

#include <delaunay_path_planner_core/Path.h>

namespace path_planner {

    Path::Path() {


    }

    Path::~Path() {

        // free the states
        for(std::shared_ptr<path_planner::State> s : this->states) {
            // free the pointer
            s.reset();
        }
    }

    void Path::addState(const std::shared_ptr<path_planner::State>& s, float cost) {

        // the state list is empty, so we can't check
        if(this->states.empty()) {
            this->states.push_back(s);
            return;
        }

        // if they aren't successors, don't allow to add
        if(!this->states.back()->hasNeighbor(s)) {
            throw std::runtime_error("You tried to add a state which is not a successor!");
        }

        this->states.push_back(s);
        this->cost += cost;
    }

    void Path::prependState(const std::shared_ptr<path_planner::State> &s, float cost) {

        if(this->states.empty()) {
            this->states.push_front(s);
            return;
        }

        if(!s->hasNeighbor(this->states.front())) {
            throw std::runtime_error("You tried to add a state which is not a successor!");
        }

        this->states.push_front(s);
        this->cost += cost;
    }

    std::list<std::shared_ptr<path_planner::State>> Path::getFullPath() const {
        return this->states;
    }
} // path_planner