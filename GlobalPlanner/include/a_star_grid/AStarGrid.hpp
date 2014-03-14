//
//  AStarGrid.hpp
//  GlobalPlanner
//
//  Created by Satya Prakash on 06/09/13.
//  Copyright (c) 2013 Satya Prakash. All rights reserved.
//

#ifndef __GlobalPlanner__aStarSS__
#define __GlobalPlanner__aStarSS__

#include <iostream>
#include <string>
#include <math.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <vector>
#include <sstream>
#include <queue>
#include <map>
#include <ctime>
#include <algorithm>
#include <iterator>
#include "a_star_grid/GridState.hpp"

namespace navigation {
    
    
    
       
    template < typename T,typename Sequence = std::vector<T> , typename CompareFunction = std::less<typename Sequence::value_type> >
    class sPriorityQueue : public std::priority_queue<T,Sequence, CompareFunction>
    {
    public:
        void clear()
        {
            this->c.clear();
        }
        void removeElement(T pos_)
        {
            for (auto iterator_=this->c.begin(); iterator_!=this->c.end(); iterator_++) {
                if ((*iterator_)==pos_) {
                    this->c.erase(iterator_);
                    return;
                }
            }
            return;
        }
    };
    
    class SdcPlanner
    {
        class Direction {
            const int x_, y_, z_;
        public:
            inline Direction(int x,int y,int z):x_(x), y_(y), z_(z)    {}
            
            inline const int& x() const  {   return x_;  }
            
            inline const int& y() const  {   return y_;  }
            
            inline const int& z() const  {   return z_;  }
            
        };
        
        cv::Mat obstacleMap,closedNodes,openNodes,directionMap,mapWithObstacles;
        Direction directionOfGrids[NUMBER_OF_DIRECTIONS] = { Direction(1, 0, 0), Direction(1, 1, 0), Direction(0, 1, 0), Direction(-1, 1, 0), Direction(-1, 0, 0), Direction(-1, -1, 0), Direction(0, -1, 0), Direction(1, -1, 0), };
        
        int xValueNeighbor,yValueNeighbor;
        int xValueP,yValueP;
        void plannerWithSimpleAstar(const State& start, const State& target);

        std::string pathToTarget;
        void reconstructPath(const GridState& current, const GridState& start, const GridState& target);
        const std::vector<GridState>& neighborNodes(const GridState& current, const GridState& target);
        void populateOpenList(sPriorityQueue<GridState>& openSet, const std::vector<GridState>& neighbors);
        void addObstacles();
        
    public:
        SdcPlanner(const GridState& locationOfBot_,const GridState& targetLocation_);
        SdcPlanner();
        void tester();
    };
}

#endif /* defined(__GlobalPlanner__aStarSS__) */
