//
//  GridState.hpp
//  GlobalPlanner
//
//  Created by Satya Prakash on 26/02/14.
//  Copyright (c) 2014 Satya Prakash. All rights reserved.
//

#ifndef __GlobalPlanner__GridState__
#define __GlobalPlanner__GridState__

#include <iostream>
#include "utils/State.hpp"

namespace navigation {
    
    
    static const int MAP_WIDTH=60;
    static const int MAP_HEIGHT=60;
    static const int IMAGE_WIDTH=600;
    static const int IMAGE_HEIGHT=600;
    static const int NUMBER_OF_DIRECTIONS=8;


    class GridState : public State {
        
        int gCostAlongFollowedPath,approxTotalCostOfPathToTarget,directionFromParent;
        
    public:
        GridState(const State& state,int gCost,int totalCost) : State(state),gCostAlongFollowedPath(gCost),approxTotalCostOfPathToTarget(totalCost) {}

        inline GridState(int xCordinate, int yCordinate, double theta, double curvature, const double g_cost, const double totalCost ) : State(xCordinate, yCordinate, theta, curvature), gCostAlongFollowedPath(g_cost), approxTotalCostOfPathToTarget(totalCost)  {}

        inline int getDirectionFromParent()      const   {   return directionFromParent;             }
        
        void setDirectionOfParent(int direction)   {   directionFromParent=direction ;         }
        
        inline const int& getGcostAlongFollowedPath()     const        {   return gCostAlongFollowedPath;          }
        
        inline const int& getApproxCostOfPathToTarget() const     {   return approxTotalCostOfPathToTarget;   }
        
        inline void updateTotalCost(const GridState& position) {
            approxTotalCostOfPathToTarget=gCostAlongFollowedPath+estimateHeuristicCostToTarget(position)*10; //A*
        }
        
        // give better priority to going strait instead of diagonally
        inline void updateGcostOfneighbour(const int& i) // i: direction
        {
            gCostAlongFollowedPath+=(NUMBER_OF_DIRECTIONS==8?(i%2==0?10:14):10);
        }
        
        bool operator==(const GridState& state) const
        {
            if (this->x() == state.x()&& this->y() == state.y() && this->theta() == state.theta())
                return true;
            return false;
        }
        
        // Estimation function for the remaining distance to the goal.
        const int estimateHeuristicCostToTarget(const GridState& state) const
        {
            static int xDifference, yDifference, distance;
            xDifference=state.x()-this->x();
            yDifference=state.y()-this->y();
            
            // Euclidian Distance
            distance=static_cast<int>(sqrt(xDifference*xDifference+yDifference*yDifference));
            
            // Manhattan distance
            //d=abs(xd)+abs(yd);
            
            // Chebyshev distance
            //d=max(abs(xd), abs(yd));
            return(distance);
        }
        
        inline bool operator<(const GridState& state ) const
        {
            return approxTotalCostOfPathToTarget > state.getApproxCostOfPathToTarget();
        }

        
        inline GridState(const State& state) : State(state) {}
        
        inline GridState(const int& x, const int& y, const double& theta = 0) : State(x, y, theta, 0)  {}
       
        GridState() = delete ;
        
        inline bool isOnTheMap()
        {
            if (!(((this->x() >= 0) && (this->x() < MAP_WIDTH)) && ((this->y() >= 0) && (this->y() < MAP_HEIGHT))))     return true;
            return false;
        }
        
        inline double costWIthSplineFitting()
        {
            return sqrt(this->x() * this->y() + this->x() * this->y());
        }
        
        int GridDistanceSqTo(const GridState& state)  ;
        
    };
}

#endif /* defined(__GlobalPlanner__GridState__) */
