/* 
 * File:   Clothoid.cpp
 * Original Version by: Shiwangi Shah
 * Re-edits by: Satya Prakash, Samuel Anudeep
 * 
 * Created on 21 December, 2013, 12:32 PM
 */

#include "road_navigation/Clothoid.hpp"
#include "road_navigation/Fresnel.hpp"

namespace navigation {

    Clothoid::Clothoid(Pose2D& start, Pose2D& goal) {
        this->start.x = start.x;
        this->start.y = start.y;
        this->start.theta = start.theta;
        this->start.curvature = 0;

        this->goal.x = goal.x;
        this->goal.y = goal.y;
        this->goal.theta = goal.theta;
        this->goal.curvature = 0;

        kMax = 1000;
    }

    Clothoid::Clothoid() {
    }

    Clothoid::Clothoid(const Clothoid& orig) {
    }

    void Clothoid::generate() {
        double k0 = 0;
        double x0, y0;
        double theta0 = start.theta;

        solveForXY(arc_length / 2, sigma / 2, k0, theta0, x0, y0);

        for (double s = 0; s < arc_length; s += arc_length / 1000) {
            double x = 0, y = 0, theta = 0;

            if (s <= arc_length / 2) {
                if (sigma * s < kMax) {
                    double a = sigma / 2, b = k0, c = theta0;
                    solveForXY(s, a, b, c, x, y);
                    theta = Pose2D::reduceTheta(sigma * s * s / 2 + theta0);
                } else {
                    x = sin(kMax * s + theta0) / kMax - sin(theta0) / kMax;
                    y = cos(theta0) / kMax - cos(kMax * s + theta0) / kMax;
                    theta = Pose2D::reduceTheta(kMax * s + theta0);
                }
            } else {
                if (sigma * s < kMax) {
                    double a = -sigma / 2;
                    double b = sigma * arc_length / 2 + k0;
                    double c = sigma / 2 * arc_length * arc_length / 4 + k0 * arc_length / 2 + theta0;
                    double x1, y1;
                    solveForXY(s - arc_length / 2, a, b, c, x1, y1);
                    solveForXY(0, a, b, c, x, y);
                    x = x0 + x1 - x;
                    y = y0 + y1 - y;
                    theta = Pose2D::reduceTheta(sigma * arc_length * s - sigma * s * s / 2 + theta0);
                } else {
                    x = sin(kMax * s + theta0) / kMax - sin(theta0) / kMax;
                    y = cos(theta0) / kMax - cos(kMax * s + theta0) / kMax;
                    theta = Pose2D::reduceTheta(kMax * s + theta0);
                }
            }

            // TODO: Package the tangent angle too
            points.push_back(new Pose2D(start.x + x, start.y + y, theta));
        }
    }

    double Clothoid::getSigma() {
        return sigma;
    }

    Clothoid::~Clothoid() {
        for (int i = 0; i < points.size(); i++) {
            delete points.at(i);
        }
        points.clear();
    }

    void Clothoid::calculateParameters() {
        start.reduceTheta();
        goal.reduceTheta();

        double alpha = Pose2D::reduceTheta(-start.theta + goal.theta) / 2;
        double abs_alpha = fabs(alpha);
        double x, z;
        Fresnel::fresnel(sqrt(2 * abs_alpha / PI), x, z);
        double D = x * cos(abs_alpha) + z * sin(abs_alpha);
        
        sigma = 4 * PI * signum(alpha) * D * D / start.distance(goal);
        arc_length = 2 * sqrt(fabs(2 * alpha / sigma));
        ROS_DEBUG("[local_planner/Clothoid/calculateParameters] sigma = %lf", sigma);
        ROS_DEBUG("[local_planner/Clothoid/calculateParameters] arc_length = %lf", arc_length);
    }

    int Clothoid::signum(double x) {
        return x > 0 ? 1 : -1;
    }

    void Clothoid::solveForXY(double s, double a, double b, double c, double& x, double& y) {
        double storeX, storeY;

        if (a > 0) {
            double limit_ = (b + 2 * a * s) / (sqrt(2 * fabs(a) * PI));
            Fresnel::fresnel(limit_, storeX, storeY);

            x = sqrt(PI / 2 / fabs(a))*((cos(b * b / 4 / a - c))*(storeX)+(sin(b * b / 4 / a - c))*(storeY));
            y = sqrt(PI / 2 / fabs(a))*((cos(b * b / 4 / a - c))*(storeY)-(sin(b * b / 4 / a - c))*(storeX));

            double x1, y1;
            limit_ = b / (sqrt(2 * fabs(a) * PI));
            Fresnel::fresnel(limit_, storeX, storeY);
            x1 = sqrt(PI / 2 / fabs(a))*((cos(b * b / 4 / a - c))*(storeX)+(sin(b * b / 4 / a - c))*(storeY));
            y1 = sqrt(PI / 2 / fabs(a))*((cos(b * b / 4 / a - c))*(storeY)-(sin(b * b / 4 / a - c))*(storeX));

            x -= x1;
            y -= y1;
        } else {
            a = -a;
            double limit_ = (2 * a * s - b) / (sqrt(2 * fabs(a) * PI));
            Fresnel::fresnel(limit_, storeX, storeY);

            x = sqrt(PI / 2 / fabs(a))*((cos(b * b / 4 / a + c))*(storeX)+(sin(b * b / 4 / a + c))*(storeY));
            y = sqrt(PI / 2 / fabs(a))*(-(cos(b * b / 4 / a + c))*(storeY)+(sin(b * b / 4 / a + c))*(storeX));

            double x1, y1;

            limit_ = (-b) / (sqrt(2 * a * PI));

            Fresnel::fresnel(limit_, storeX, storeY);

            x1 = sqrt(PI / 2 / fabs(a))*((cos(b * b / 4 / a + c))*(storeX)+(sin(b * b / 4 / a + c))*(storeY));
            y1 = sqrt(PI / 2 / fabs(a))*(-(cos(b * b / 4 / a + c))*(storeY)+(sin(b * b / 4 / a + c))*(storeX));
            x -= x1;
            y -= y1;
        }
    }
}
