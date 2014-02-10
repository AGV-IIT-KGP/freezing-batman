/* 
 * File:   Clothoid.hpp
 * Author: Shiwangi
 *
 * Created on 23 January, 2014, 6:23 PM
 */

#ifndef CLOTHOID_HPP
#define	CLOTHOID_HPP

#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <cmath>
#include <vector>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <road_navigation/State.h>
#include <road_navigation/ClothoidPathSegment.h>

static const double PI = 3.14159;
static const int HEIGHT = 800, WIDTH = 800;
using namespace std;

class Clothoid {
public:
    int kMax;
    int solution;
    vector<ClothoidPathSegment> paths;
    
    Clothoid();
    void getPath(State start, State end);
    
private:
    bool debug;
    PathSegment path;
    State start, end;

    double calcD(double alpha);
    int fresnel(double x, double &costerm, double &sinterm);
    void getTrajectory();
    void getXY(double s, double a, double b, double c, double& x, double& y);
    double inRange(double theta);
    int signum(double a);
};



#endif	/* CLOTHOID_HPP */

