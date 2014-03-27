/* 
 * File:   Clothoid.hpp
 * Author: samuel
 *
 * Created on 21 December, 2013, 12:32 PM
 */

#ifndef CLOTHOID_HPP
#define	CLOTHOID_HPP

#include <ros/ros.h> // To be removed when debugging support is available in agv_framework
#include <cmath>
#include <iostream>
#include <vector>
#include "Posture.hpp"
#include "utils/Pose2D.hpp"

namespace navigation {

    class Clothoid {
    public:
        std::vector<Pose2D*> points;

        Clothoid(Pose2D& start, Pose2D& goal);
        Clothoid();
        Clothoid(const Clothoid& orig);
        void calculateParameters();
        void generate();
        double getSigma();
        virtual ~Clothoid();
    private:
        double sigma;
        double arc_length;
        double kMax;
        Posture start, goal;

        int signum(double x);
        void solveForXY(double s, double a, double b, double c, double& x, double& y);
    };
}

#endif	/* CLOTHOID_HPP */

