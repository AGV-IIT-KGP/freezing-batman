/* 
 * File:   Pose2D.hpp
 * Author: samuel
 *
 * Created on 21 December, 2013, 12:34 PM
 */

#ifndef POSE2D_HPP
#define	POSE2D_HPP

#define PI 3.141

class Pose2D {
public:
    double x, y, theta;

    Pose2D(double x, double y, double theta);
    Pose2D();
    Pose2D(const Pose2D& orig);
    virtual ~Pose2D();

    double distance(Pose2D& other);
    static double reducedATan(double theta, Pose2D& a, Pose2D& b);
    static double reduceTheta(double theta);
    void reduceTheta();

private:

};

#endif	/* POSE2D_HPP */

