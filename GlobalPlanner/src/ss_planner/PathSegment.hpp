//
//  PathSegment.hpp
//  GlobalPlanner
//
//  Created by Satya Prakash on 28/01/14.
//  Copyright (c) 2014 Satya Prakash. All rights reserved.
//

#ifndef __GlobalPlanner__PathSegment__
#define __GlobalPlanner__PathSegment__


#include <vector>
#include <list>
#include <forward_list>
#include "Pose.hpp"
#include "State.hpp"
#include "SSState.hpp"

namespace navigation {
    
    class PathSegment   {
     
        const double lengthOfPathSegment_;
        std::vector<SSState> pathSegmentPoints_;
    public:
        PathSegment(const double lengthOfPathSegment, const std::vector<SSState>& pathSegmentPoints):lengthOfPathSegment_(lengthOfPathSegment), pathSegmentPoints_(pathSegmentPoints){}
        
        inline std::vector<SSState> states(){
            return pathSegmentPoints_;
        }
    };
    
    typedef std::shared_ptr<PathSegment> PathSegmentPtr;
}

#endif /* defined(__GlobalPlanner__PathSegment__) */
