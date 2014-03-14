//
//  Path.hpp
//  GlobalPlanner
//
//  Created by Satya Prakash on 28/01/14.
//  Copyright (c) 2014 Satya Prakash. All rights reserved.
//

#ifndef __GlobalPlanner__Path__
#define __GlobalPlanner__Path__

#include <iostream>

namespace navigation    {
    
    class Path  {
        
        const double cost_;
        
        Path(const double cost):cost_(cost){}
        
    };
    
    typedef std::shared_ptr<Path> PathPtr;
    
}

#endif /* defined(__GlobalPlanner__Path__) */
