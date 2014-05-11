//
//  state.hpp
//  AStarSeed
//
//  Created by Satya Prakash on 28/01/14.
//  Copyright (c) 2014 Satya Prakash. All rights reserved.
//

#ifndef __AStarSeed__State__
#define __AStarSeed__State__

#include <cmath>
#include <string>
#include <iostream>

namespace navigation    {
    
    class State {
        
        // TODO: find a way make it immutable
        // load in swapping after making these const
        int xCordinate_ , yCordinate_;
        int theta_, curvature_ ;
        
        static const double inRange(const double theta_) ;
        
    public:
        inline int x()         const       { return xCordinate_;   }
        inline int y()         const       { return yCordinate_;   }
        inline int theta()     const       { return theta_;        }
        inline int curvature() const       { return curvature_;    }
        
        //TODO : removue this
        inline State(){    }
        inline State(int xCordinate, int yCordinate, int theta, int curvature ) : xCordinate_(xCordinate), yCordinate_(yCordinate), theta_(theta), curvature_(curvature)    {}
        
        inline double distanceSqTo(const State& b) const    {
            
            return (xCordinate_ - b.xCordinate_) * (xCordinate_ - b.xCordinate_) + (yCordinate_ - b.yCordinate_) * (yCordinate_ - b.yCordinate_);
            
        }
        
        inline double distanceTo(const State& b) const {
            
            return sqrt((xCordinate_ - b.xCordinate_) * (xCordinate_ - b.xCordinate_) + (yCordinate_ - b.yCordinate_) * (yCordinate_ - b.yCordinate_));
        
            
        }
        
        inline double euclidianDistanceTo(const State& b) const {
            
            return sqrt((xCordinate_ - b.xCordinate_) * (xCordinate_ - b.xCordinate_) + (yCordinate_ - b.yCordinate_) * (yCordinate_ - b.yCordinate_));
            
            
        }
        
        inline double manhattanDistanceTo(const State& b) const {
            
            return std::abs(xCordinate_ - b.xCordinate_) + std::abs(yCordinate_ - b.yCordinate_);
            
            
        }
        
        inline bool operator==(const State& b) const    {
            return xCordinate_==b.x() && yCordinate_==b.y() && theta_==b.theta() && curvature_==curvature();
        }
    	
		inline bool operator!=(const State& b) const    {
            return !(xCordinate_==b.x() && yCordinate_==b.y() && theta_==b.theta() && curvature_==curvature());
        }
        
        const std::string toString() const;
        
//        bool isOutsideOfMap()
//        {
//            if (!(((xCordinate_ >= 0) && (xCordinate_ < MAP_MAX)) &&
//                  ((yCordinate_ >= 0) && (yCordinate_ < MAP_MAX))))
//                return true;
//            else
//                return false;
//        }
//        

        
        
//        const std::ostream& operator<<(const std::ostream& os, const State& state);
        
    };
    
    // typedef std::shared_ptr<State> StatePtr;
    typedef State* StatePtr;
}


#endif /* defined(__AStarSeed__State__) */
