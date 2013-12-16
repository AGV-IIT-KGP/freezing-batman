/* 
 * File:   State.hpp
 * Author: satya
 *
 * Created on December 13, 2013, 5:56 PM
 */

#ifndef STATE_HPP
#define	STATE_HPP

namespace navigation {

    class State {
    public:
        State(const State& orig);
        virtual ~State();
        double x, y, theta, curvature;
        State(double x_, double y_, double theta_, double curvature_);
        State(double x_, double y_, double theta_);
        State();
        double distance(State b);
    private:

    };
}

#endif	/* STATE_HPP */

