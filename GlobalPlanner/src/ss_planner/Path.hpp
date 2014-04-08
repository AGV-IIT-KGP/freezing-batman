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
#include <vector>
#include "PathSegment.hpp"

namespace navigation    {
    
    class Path  {
        
        const double cost_;
        std::vector<PathSegmentPtr> segments_;
        
    public:
        Path(const double cost, const std::vector<PathSegmentPtr>& segments):cost_(cost), segments_(segments){}
        inline double cost(){
            return cost_;
        }

        inline std::vector<PathSegmentPtr> segments(){
            return segments_;
        }
        
    };
    
    typedef std::shared_ptr<Path> PathPtr;
    
}

#endif /* defined(__GlobalPlanner__Path__) */
