/* 
 * File:   Trajectory.cpp
 * Author: samuel
 * 
 * Created on 21 December, 2013, 12:31 PM
 */

#include "road_navigation/Trajectory.hpp"

namespace navigation {

    Trajectory::Trajectory() {
    }

    Trajectory::Trajectory(const Trajectory& orig) {
    }

    void Trajectory::generate() {
        ROS_DEBUG("[local_planner/Trajectory/generate] start = (%lf, %lf, %lf)", start.x, start.y, start.theta);
        ROS_DEBUG("[local_planner/Trajectory/generate] goal = (%lf, %lf, %lf)", goal.x, goal.y, goal.theta);

        double beta = atan((goal.y - start.y) / (goal.x - start.x));

        if (fabs(beta - goal.theta - start.theta + beta) < 0.001) {
            ROS_DEBUG("[local_planner/Trajectory/generate] case = elementary curve");
            generateSegment(start, goal);
        } else {
            Pose2D intermediate(0, 0, 0);

            if (fabs(start.theta - goal.theta) < 0.001) {
                ROS_DEBUG("[local_planner/Trajectory/generate] case = parallel postures");

                intermediate.x = (start.x + goal.x) / 2;
                intermediate.y = (start.y + goal.y) / 2;
                intermediate.theta = 2 * atan((intermediate.y - start.y) / (intermediate.x - start.x)) - start.theta;
            } else {
                ROS_DEBUG("[local_planner/Trajectory/generate] case = arbitrary postures");

                double alpha = (-start.theta + goal.theta) / 2;
                // Center of the circle: the locus of the point q between which the 2 eulers are drawn.
                Pose2D center(0, 0, 0);
                center.x = (start.x + goal.x + (start.y - goal.y) * cos(alpha) / sin(alpha)) / 2;
                center.y = (start.y + goal.y + (goal.x - start.x) * cos(alpha) / sin(alpha)) / 2;
                ROS_DEBUG("[local_planner/Trajectory/generate] center = (%lf, %lf)", center.x, center.y);
                double radius = sqrt(center.distance(start));
                ROS_DEBUG("[local_planner/Trajectory/generate] radius = %lf", radius);
                double deflection1 = Pose2D::reducedATan(atan((center.y - start.y) / (center.x - start.x)), center, start);
                double deflection2 = Pose2D::reducedATan(atan((center.y - goal.y) / (center.x - goal.x)), center, goal);
                if (deflection2 < deflection1) {
                    //swap
                    double temp = deflection2;
                    deflection2 = deflection1;
                    deflection1 = temp;
                }
                double deflection = (deflection2 - deflection1) / 2 + deflection1;
                ROS_DEBUG("[local_planner/Trajectory/generate] deflections = (%lf, %lf, %lf)", deflection1, deflection2, deflection);

                intermediate.x = center.x + radius * cos(deflection);
                intermediate.y = center.y + radius * sin(deflection);
                intermediate.theta = 2 * atan((intermediate.y - start.y) / (intermediate.x - start.x)) - start.theta;
            }

            ROS_DEBUG("[local_planner/Trajectory/generate] intermediate = (%lf, %lf, %lf)", intermediate.x, intermediate.y, intermediate.theta);

            generateSegment(start, intermediate);
            generateSegment(intermediate, goal);
        }
    }

    void Trajectory::setGoal(double x, double y, double theta) {
        goal.x = x;
        goal.y = y;
        goal.theta = theta;
    }

    void Trajectory::setStart(double x, double y, double theta) {
        start.x = x;
        start.y = y;
        start.theta = theta;
    }

    Trajectory::~Trajectory() {
        for (int i = 0; i < segments.size(); i++) {
            delete segments.at(i);
        }
    }

    void Trajectory::generateSegment(Pose2D& start, Pose2D& goal) {
        Clothoid *elementary_curve = new Clothoid(start, goal);
        elementary_curve->calculateParameters();
        elementary_curve->generate();
        segments.push_back(elementary_curve);
    }
}

