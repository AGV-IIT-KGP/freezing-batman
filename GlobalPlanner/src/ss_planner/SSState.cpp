//
//  SSState.cpp
//  GlobalPlanner
//
//  Created by Satya Prakash on 28/01/14.
//  Copyright (c) 2014 Satya Prakash. All rights reserved.
//

#include "ss_planner/SSState.hpp"
#include <iostream>
namespace navigation {
    
    const SSState& SSState::operator+(const Seed& seed) const {
        
        return std::move(SSState(x()+seed.x_,y()+seed.y_,theta()+seed.finalTheta_-M_PI_2,0, g_cost_+seed.cost_,0));
    }

}

