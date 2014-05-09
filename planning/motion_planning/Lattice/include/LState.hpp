#ifndef __Lattice__LState__
#define __Lattice__LState__

#include <cmath>
#include <string>
#include <iostream>

namespace navigation    {
    
    class LState {
        
        int xCordinate_ , yCordinate_;
        int theta_, curvature_ ;
        
        static const double inRange(const double theta_) ;
        
    public:
        inline int x()         const       { return xCordinate_;   }
        inline int y()         const       { return yCordinate_;   }
        inline int theta()     const       { return theta_;        }
        inline int curvature() const       { return curvature_;    }
        
        //TODO : removue this
        inline LState(){    }
        inline LState(int xCordinate, int yCordinate, int theta, int curvature ) : xCordinate_(xCordinate), yCordinate_(yCordinate), theta_(theta), curvature_(curvature) {}
        
        inline double distanceSqTo(const LState& b) const    {
            
            return (xCordinate_ - b.xCordinate_) * (xCordinate_ - b.xCordinate_) + (yCordinate_ - b.yCordinate_) * (yCordinate_ - b.yCordinate_);
            
        }
        
        inline double distanceTo(const LState& b) const {
            
            return sqrt((xCordinate_ - b.xCordinate_) * (xCordinate_ - b.xCordinate_) + (yCordinate_ - b.yCordinate_) * (yCordinate_ - b.yCordinate_));
        
            
        }
        
        inline double euclidianDistanceTo(const LState& b) const {
            
            return sqrt((xCordinate_ - b.xCordinate_) * (xCordinate_ - b.xCordinate_) + (yCordinate_ - b.yCordinate_) * (yCordinate_ - b.yCordinate_));
            
            
        }
        
        inline bool operator==(const LState& b) const    {
            return xCordinate_==b.x() && yCordinate_==b.y() && theta_==b.theta() && curvature_==b.curvature();
        }
    	
		inline bool operator!=(const LState& b) const    {
            return !(xCordinate_==b.x() && yCordinate_==b.y() && theta_==b.theta() && curvature_==b.curvature());
        }
        
        const std::string toString() const;
#ifndef // __Lattice__LState__