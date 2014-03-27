/* 
 * File:   Posture.hpp
 * Author: samuel
 *
 * Created on 21 December, 2013, 12:33 PM
 */

#ifndef POSTURE_HPP
#define	POSTURE_HPP

#include "utils/Pose2D.hpp"

namespace navigation {

    class Posture : public Pose2D {
    public:
        double curvature;

        Posture(double x, double y, double theta, double curvature);
        Posture();
        Posture(const Posture& orig);
        virtual ~Posture();
    private:

    };
}

#endif	/* POSTURE_HPP */

