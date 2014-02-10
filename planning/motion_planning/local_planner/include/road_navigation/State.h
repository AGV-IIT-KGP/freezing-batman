/* 
 * File:   State.h
 * Author: Shiwangi
 *
 * Created on 23 January, 2014, 6:24 PM
 */

#ifndef STATE_H
#define	STATE_H

class State {
public:
    double x, y, theta, curvature;
    State(const double x_, const double y_, const double theta_, const double curvature_);
    State(double x_, double y_, double theta_);
    State();
    double getDistance(State b);
//    State& operator=(const State a);

};




#endif	/* STATE_H */

