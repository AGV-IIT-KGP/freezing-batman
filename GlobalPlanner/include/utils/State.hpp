//
//  State.h
//  GlobalPlanner
//
//  Created by Satya Prakash on 28/01/14.
//  Copyright (c) 2014 Satya Prakash. All rights reserved.
//

#ifndef __GlobalPlanner__State__
#define __GlobalPlanner__State__

#include <cmath>
#include <string>
#include <iostream>

namespace navigation    {
    
    class State {
        
        // TODO: find a way make it immutable
        // load in swapping after making these const
        int xCordinate_, yCordinate_;
        double theta_, curvature_;
        
        static const double inRange(const double theta_) ;//why const return?
        
    public:
        inline const int x()         const       { return xCordinate_;   }
        inline const int y()         const       { return yCordinate_;   }
        inline const double theta()     const       { return theta_;        }
        inline const double curvature() const       { return curvature_;    }
        
        //TODO : removue this
        inline State(){    }
        inline State(int xCordinate, int yCordinate, double theta, double curvature ) : xCordinate_{xCordinate}, yCordinate_{yCordinate}, theta_{inRange(theta)}, curvature_{curvature}    {}
        
        inline double distanceSqTo(const State& b) const    {
            
            return (xCordinate_ - b.xCordinate_) * (xCordinate_ - b.xCordinate_) + (yCordinate_ - b.yCordinate_) * (yCordinate_ - b.yCordinate_);
            
        }
        
        inline const double distanceTo(const State& b) const {
            
            return sqrt(xCordinate_ - b.xCordinate_) * (xCordinate_ - b.xCordinate_) + (yCordinate_ - b.yCordinate_) * (yCordinate_ - b.yCordinate_);
        
            
        }
        
        inline bool operator==(const State& b) const    {
            return xCordinate_==b.x() && yCordinate_==b.y() && theta_==b.theta() && curvature_==curvature();
        }
        
        const std::string toString() const;
        
//        const std::ostream& operator<<(const std::ostream& os, const State& state);
        
    };
    
    typedef std::shared_ptr<State> StatePtr;
}


#endif /* defined(__GlobalPlanner__State__) */
