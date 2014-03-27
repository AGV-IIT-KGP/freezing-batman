//
//  Seed.hpp
//  GlobalPlanner
//
//  Created by Satya Prakash on 20/02/14.
//  Copyright (c) 2014 Satya Prakash. All rights reserved.
//

#ifndef __GlobalPlanner__Seed__
#define __GlobalPlanner__Seed__

#include <iostream>

namespace navigation {
    
    class Seed  {
    public:
        double x_, y_, theta_, cost_;
        Seed(){}
        Seed(const double x,const double y,const double theta, double cost) : x_(x), y_(y), theta_(theta), cost_(cost) {}
    };
    
}

#endif /* defined(__GlobalPlanner__Seed__) */
