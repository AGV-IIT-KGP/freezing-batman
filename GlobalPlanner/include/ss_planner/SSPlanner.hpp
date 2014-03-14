//
//  SSPlanner.hpp
//  GlobalPlanner
//
//  Created by Satya Prakash on 28/01/14.
//  Copyright (c) 2014 Satya Prakash. All rights reserved.
//

#ifndef __GlobalPlanner__SSPlanner__
#define __GlobalPlanner__SSPlanner__

#include "global_planner/GlobalPlanner.hpp"
#include "ss_planner/SSPath.hpp"
#include "ss_planner/SSState.hpp"
#include "ss_planner/SSPriority_queue.hpp"
#include "utils/dyarray.hpp"
#include "utils/PathPlanner.hpp"
#include <unordered_map>
#include <opencv/cv.hpp>
#include "utils/Seed.hpp"

static const unsigned int NO_OF_NEIGHBORS = 11;
static const unsigned int NO_OF_SEEDS = 5;


namespace navigation {
    

    class SSPlanner : public PathPlanner {
        

        cv::Mat img;
        SS::dyarray<Seed> seeds;
        
//        static const unsigned int NO_OF_NEIGHBORS = 11;

    public:
        // TODO : function will be const or not
        
        SSPlanner();
        
        const PathPtr traversablePath(const State& start, const State& goal) const;
        const double heuristicCost(const SSState& start, const SSState& goal) const;
        const std::array<SSState, NO_OF_NEIGHBORS>& neighborNodes(const SSState& current) const;
        const PathPtr reconstructPath(const std::unordered_map<SSState, SSState>& came_from, const SSState& goal) const;
        void loadSeeds() const;
    };
    
    typedef std::shared_ptr<SSPlanner> SSPlannerPtr;
    
}

#endif /* defined(__GlobalPlanner__SSPlanner__) */
