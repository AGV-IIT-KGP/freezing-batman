/* 
 * File:   Fresnel.hpp
 * Author: samuel
 *
 * Created on 21 December, 2013, 1:45 PM
 */

#ifndef FRESNEL_HPP
#define	FRESNEL_HPP

#include <cmath>
#include <iostream>

namespace navigation {

    class Fresnel {
    public:
        static void fresnel(double x, double &cos_term, double &sin_term);
        Fresnel();
        Fresnel(const Fresnel& orig);
        virtual ~Fresnel();
    private:
        static int signum(double x);
    };
}

#endif	/* FRESNEL_HPP */

