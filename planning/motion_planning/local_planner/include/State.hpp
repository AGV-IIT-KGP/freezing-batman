/* 
 * File:   State.hpp
 * Author: Satya Prakash
 *
 * Created on December 13, 2013, 5:56 PM
 */

#ifndef STATE_HPP
#define	STATE_HPP

namespace navigation {

    class State {
    public:
        double x, y, theta, curvature;
        State(double x_, double y_, double theta_, double curvature_) : x(x_), y(y_), theta(theta_), curvature(curvature_)  {}
        State(double x_, double y_, double theta_) :  x(x_), y(y_), theta(theta_) {}
        inline double distanceSq(const State b) const {   return (x - b.x) * (x - b.x) + (y - b.y) * (y - b.y);    }

    };
}

#endif	/* STATE_HPP */

