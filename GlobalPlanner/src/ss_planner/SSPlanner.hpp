//
//  SSPlanner.hpp
//  GlobalPlanner
//
//  Created by Satya Prakash on 28/01/14.
//  Copyright (c) 2014 Satya Prakash. All rights reserved.
//

#ifndef __GlobalPlanner__SSPlanner__
#define __GlobalPlanner__SSPlanner__

#include "SSPath.hpp"
#include "SSState.hpp"
#include "SSPathSegment.hpp"
#include "SSPriority_queue.hpp"
#include "dyarray.hpp"
#include "PathPlanner.hpp"
#include <unordered_map>
#include <opencv2/cv.h>
#include <OpenCV2/highgui.h>
#include "Seed.hpp"
#include <array>

static const unsigned int NO_OF_NEIGHBORS = 8;
static const unsigned int NO_OF_SEEDS = 8;
static const unsigned int IMG_HEIGHT = 800, IMG_WIDTH = 800;
static const int seedLength = 10;

namespace navigation {
    
    
    enum class Heuristic    {
        SS_HEUCLIDIAN, SS_HMANHATTAN,SS_TEMP
    };

    class SSPlanner : public PathPlanner {
        

        mutable cv::Mat img;
        std::array<Seed, NO_OF_SEEDS> seeds;
        
//        static const unsigned int NO_OF_NEIGHBORS = 11;

    public:
        // TODO : function will be const or not
        
        SSPlanner();
        void showPath(PathPtr path) const;
        bool isCloseTo(const SSState& current, const SSState& goal) const;
        PathPtr traversablePath(const State& start, const State& goal) const;
        double heuristicCost(const SSState& start, const SSState& goal, Heuristic hueristic) const;
        std::array<SSState, NO_OF_NEIGHBORS> neighborNodes(const SSState& current, const SSState& goal) const;
        PathPtr reconstructPath(std::unordered_map<SSState, SSState>& came_from, const SSState& current, const SSState& start) const;
        void loadSeeds() const;
    };
    
    typedef std::shared_ptr<SSPlanner> SSPlannerPtr;
    
}

#endif /* defined(__GlobalPlanner__SSPlanner__) */
