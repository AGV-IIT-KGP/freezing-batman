//
//  StateOfCar.hpp
//  AStarSeed
//
//  Created by Satya Prakash on 27/03/14.
//  Copyright (c) 2014 Satya Prakash. All rights reserved.
//

#ifndef __AStarSeed__StateOfCar__
#define __AStarSeed__StateOfCar__

#include <iostream>
#include <State/State.hpp>



namespace navigation {
    
    
    static const int MAP_MAX = 800;
    
    class StateOfCar : public State
    {
        double g_cost_, h_cost_ , f_cost_ ;
        int seedTaken_;
    public:
        // remove this constructor
        inline StateOfCar():State(){}
        inline StateOfCar(const State& state) : State(state)   {}
        
//        inline StateOfCar(const StateOfCar& state) : State(state.x(), state.y(), state.theta(), state.curvature()), g_cost_(state.gCost()), h_cost_(state.hCost()), f_cost_(state.gCost()+state.hCost())  {}
        
        inline StateOfCar(const State& state,const double g_cost, const double h_cost, int seedTakenP) : State(state), g_cost_(g_cost), h_cost_(h_cost) , f_cost_(g_cost+h_cost), seedTaken_(seedTakenP) {}
        
        inline StateOfCar(int xCordinate, int yCordinate, double theta, double curvature, const double g_cost, const double h_cost , int seedTakenP) : State(xCordinate, yCordinate, theta, curvature), g_cost_(g_cost), h_cost_(h_cost) , f_cost_(g_cost+h_cost) , seedTaken_(seedTakenP) {}
        
        
        inline double gCost()     const   {   return g_cost_;              }
        inline double hCost()     const   {   return h_cost_;              }
        inline double totalCost() const   {   return f_cost_;    }
        inline double seedTaken() const   {   return seedTaken_;    }
        
        inline void gCost(const double g_cost)          {   g_cost_ = g_cost;                       }
        inline void hCost(const double h_cost)          {   h_cost_ = h_cost;                       }
        inline void updateTotalCost()             {   f_cost_ = g_cost_ + h_cost_;  }

        
        inline bool operator==(const StateOfCar& b) const  {
            
            return x()==b.x() && y()==b.y();
        }
        
        inline bool operator()(const StateOfCar& b) const  {
//            return f_cost_ < b.totalCost();
            double k11 = x();
            double k12 = y();
            double k13 = theta();
            
            double cantor11 = 0.5 * (k11 + k12) * (k11 + k12 + 1) + k12;
            double cantor12 = 0.5 * (cantor11 + k13) * (cantor11 + k13 + 1) + k13;
            
            double k21 = b.x();
            double k22 = b.y();
            double k23 = b.theta();
            
            double cantor21 = 0.5 * (k21 + k22) * (k21 + k22 + 1) + k22;
            double cantor22 = 0.5 * (cantor21 + k23) * (cantor21 + k23 + 1) + k23;
            
            return cantor12 < cantor22;

        }
        
        inline bool operator<(const StateOfCar& b) const   {
            
            double k11 = x();
            double k12 = y();
            double k13 = theta();
            
            double cantor11 = 0.5 * (k11 + k12) * (k11 + k12 + 1) + k12;
            double cantor12 = 0.5 * (cantor11 + k13) * (cantor11 + k13 + 1) + k13;
            
            double k21 = b.x();
            double k22 = b.y();
            double k23 = b.theta();
            
            double cantor21 = 0.5 * (k21 + k22) * (k21 + k22 + 1) + k22;
            double cantor22 = 0.5 * (cantor21 + k23) * (cantor21 + k23 + 1) + k23;
            
            return cantor12 < cantor22;
//            return f_cost_ > b.totalCost();
        }
        
//        StateOfCar operator+(const Seed& seed) const;

        
        inline bool isCloseTo(StateOfCar const& givenLocation){
            
            return (distanceTo(givenLocation) < 250);
        }
        bool isSameAs(StateOfCar const& givenLocation){

            return (distanceTo(givenLocation) < 35);
        }
        
        bool isOutsideOfMap()
        {
            if (!(((x() >= 0) && (x() < MAP_MAX)) &&
                  ((y() >= 0) && (y() < MAP_MAX))))
                return true;
            else
                return false;
        }
        
        
    };
    
}
#endif /* defined(__AStarSeed__StateOfCar__) */
