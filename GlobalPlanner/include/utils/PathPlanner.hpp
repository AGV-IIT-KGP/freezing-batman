//
//  PathPlanner.hpp
//  GlobalPlanner
//
//  Created by Satya Prakash on 19/02/14.
//  Copyright (c) 2014 Satya Prakash. All rights reserved.
//

#ifndef __GlobalPlanner__PathPlanner__
#define __GlobalPlanner__PathPlanner__

#include <iostream>
#include "Path.hpp"
#include "State.hpp"

namespace navigation {
    
    class PathPlanner   {
        
    public:
        
        virtual const PathPtr traversablePath(const State& start, const State& goal)const = 0;

    };
    
    typedef std::shared_ptr<PathPlanner> PathPlannerPtr;
}
#endif /* defined(__GlobalPlanner__PathPlanner__) */
