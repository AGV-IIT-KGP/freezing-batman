//
//  SSState.cpp
//  GlobalPlanner
//
//  Created by Satya Prakash on 28/01/14.
//  Copyright (c) 2014 Satya Prakash. All rights reserved.
//

#include "SSState.hpp"
#include <iostream>
namespace navigation {
    
    SSState SSState::operator+(const Seed& seed) const {
        
        return SSState(x()+seed.x_,y()+seed.y_,theta()+seed.theta_,0, g_cost_+seed.cost_,0);
    }

}

