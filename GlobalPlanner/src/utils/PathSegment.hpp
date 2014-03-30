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

namespace navigation {
    
    class PathSegment   {
     
        const double lengthOfPathSegment_;
        std::forward_list<Pose> pathSegmentPoints;
        PathSegment(const double lengthOfPathSegment):lengthOfPathSegment_(lengthOfPathSegment){}
    };
    
    typedef std::shared_ptr<PathSegment> PathSegmentPtr;
}

#endif /* defined(__GlobalPlanner__PathSegment__) */
