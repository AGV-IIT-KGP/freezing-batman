/* 
 * File:   Pose2D.cpp
 * Author: samuel
 * 
 * Created on 21 December, 2013, 12:34 PM
 */

#include "utils/Pose2D.hpp"

Pose2D::Pose2D(double x, double y, double theta) {
    this->x = x;
    this->y = y;
    this->theta = theta;
}

Pose2D::Pose2D() {
}

Pose2D::Pose2D(const Pose2D& orig) {
}

Pose2D::~Pose2D() {
}

double Pose2D::distance(Pose2D& other) {
    return (x - other.x) * (x - other.x) + (y - other.y) * (y - other.y);
}

double Pose2D::reducedATan(double theta, Pose2D& start, Pose2D& goal) {
    if ((goal.x - start.x) < 0) {
        if ((goal.y - start.y) < 0) {
            theta = PI + theta;
        } else {
            theta += PI;
        }
    } else if (theta < 0) {
        theta += 2 * PI;
    }

    return theta;
}

double Pose2D::reduceTheta(double theta) {
    if (theta > PI) {
        while (theta > PI) {
            theta -= 2 * PI;
        }
    }

    if (theta <= PI) {
        while (theta <= -PI) {
            theta += 2 * PI;
        }
    }
    
    return theta;
}

void Pose2D::reduceTheta() {
    theta = reduceTheta(theta);
}

