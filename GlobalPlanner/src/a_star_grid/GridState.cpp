//
//  GridState.cpp
//  GlobalPlanner
//
//  Created by Satya Prakash on 26/02/14.
//  Copyright (c) 2014 Satya Prakash. All rights reserved.
//

#include "a_star_grid/GridState.hpp"

namespace navigation {
    
    int GridState::GridDistanceSqTo(const GridState& state)
    {
        int gridX,gridY;
        gridX=(int)(x()/MAP_WIDTH)+1;
        gridY=(int)(y()/MAP_HEIGHT)+1;
        
        int targetGridX,targetGridY;
        targetGridX=(int)(state.x()/MAP_WIDTH)+1;
        targetGridY=(int)(state.x()/MAP_HEIGHT)+1;
        
        return ((targetGridX-gridX)*(targetGridX-gridX)+(targetGridY-gridY)*(targetGridY-gridY));
    }
    
}