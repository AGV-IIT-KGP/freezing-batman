//
//  SSState.hpp
//  GlobalPlanner
//
//  Created by Satya Prakash on 28/01/14.
//  Copyright (c) 2014 Satya Prakash. All rights reserved.
//

#ifndef __GlobalPlanner__SSState__
#define __GlobalPlanner__SSState__

#include "utils/State.hpp"
#include "utils/Seed.hpp"
#include <functional>

namespace navigation {
    
    
    class SSState : public State  {
        
        double g_cost_, h_cost_, f_cost_;
        
    public:
        // remove this constructor
        inline SSState():State(){        }
        inline SSState(const State& state) : State(state)   {}
        inline SSState(const SSState& state) : State(state.x(), state.y(), state.theta(), state.curvature()), g_cost_(state.gCost()), h_cost_(state.hCost()), f_cost_(state.gCost()+state.hCost())  {}

        inline SSState(const State& state,const double g_cost, const double h_cost ) : State(state), g_cost_(g_cost), h_cost_(h_cost) , f_cost_(g_cost+h_cost)  {}
    
        inline SSState(int xCordinate, int yCordinate, double theta, double curvature, const double g_cost, const double h_cost ) : State(xCordinate, yCordinate, theta, curvature), g_cost_(g_cost), h_cost_(h_cost) , f_cost_(g_cost+h_cost)  {}
        
        
        inline const double gCost()     const   {   return g_cost_;              }
        inline const double hCost()     const   {   return h_cost_;              }
        inline const double totalCost() const   {   return f_cost_;    }
        
        inline void gCost(const double g_cost)          {   g_cost_ = g_cost;                       }
        inline void hCost(const double h_cost)          {   h_cost_ = h_cost;                       }
        inline void updateTotalCost()             {   f_cost_ = g_cost_ + h_cost_;  }
        
        inline bool operator()(const SSState& b) const  {
            return f_cost_ < b.totalCost();
        }
        
        inline bool operator<(const SSState& b) const   {
            return f_cost_ < b.totalCost();
        }
        
        const SSState& operator+(const Seed& seed) const;
    };
    
    
    typedef std::shared_ptr<SSState> SSStatePtr;
//    typedef State SSState;
//    typedef StatePtr SSStatePtr;
    
}

namespace std {
    
    
    template<> struct hash<navigation::SSState> {
        size_t operator()(const navigation::SSState& state) const {
            return std::hash<double>()(state.x()) ^ std::hash<double>()(state.y()) ^ std::hash<double>()(state.theta()) & std::hash<double>()(state.curvature());
        }
    };
}

#endif /* defined(__GlobalPlanner__SSState__) */
