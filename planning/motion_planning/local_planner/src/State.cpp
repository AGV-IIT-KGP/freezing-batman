/* 
 * File:   State.cpp
 * Author: satya
 * 
 * Created on December 13, 2013, 5:56 PM
 */

#include "State.hpp"

namespace navigation {

    State::State(double x_, double y_, double theta_, double curvature_) {
        x = x_;
        y = y_;
        theta = theta_;
        curvature = curvature_;
    }

    State::State(double x_, double y_, double theta_) {
        x = x_;
        y = y_;
        theta = theta_;
    }

    State::State() {
        x = 0;
        y = 0;
        theta = 0, curvature = 0;
    }

    double State::distance(State b) {
        return (x - b.x) * (x - b.x) + (y - b.y) * (y - b.y);
    }

    State::State(const State& orig) {
    }

    State::~State() {
    }
}