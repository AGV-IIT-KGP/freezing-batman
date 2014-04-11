//
//  state.cpp
//  AStarSeed
//
//  Created by Satya Prakash on 28/01/14.
//  Copyright (c) 2014 Satya Prakash. All rights reserved.
//

#include "a_star_seed/state.hpp"
#include <sstream>

namespace navigation {
    
    const double State::inRange(const double thetaP) {
    
        double theta = thetaP;
        while (theta >= M_PI*2 || theta < 0) {
            if (theta >= M_PI*2)
                theta -= M_PI*2;
            else
                theta += M_PI*2;
        }
        
        return theta;
    }
    

    
    const std::string State::toString() const {
        
        std::stringstream ss;
        ss<<"X Cordinate : "<<xCordinate_<<" Y Cordinate : "<<yCordinate_<<" theta : "<<theta_<<std::endl;
        return ss.str();
    }

}