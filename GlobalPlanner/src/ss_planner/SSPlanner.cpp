//
//  SSPlanner.cpp
//  GlobalPlanner
//
//  Created by Satya Prakash on 28/01/14.
//  Copyright (c) 2014 Satya Prakash. All rights reserved.
//

#include "ss_planner/SSPlanner.hpp"
#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <functional>
#include <array>

#include "ss_planner/SSPriority_queue.hpp"

namespace navigation {
    
    
    SSPlanner::SSPlanner()  {

        img = cv::Mat{cvSize(IMG_WIDTH, IMG_HEIGHT),CV_8UC3,cvScalarAll(0)};
        seeds = SS::dyarray<Seed>(NO_OF_SEEDS);
        
        seeds[0] = Seed(-4, 0, -M_PI, 4);
        seeds[1] = Seed(-4, 4, -3/4*M_PI, 4*sqrt(2));
        seeds[2] = Seed(0, 4, -M_PI_2, 4);
        seeds[3] = Seed(4, 4, -M_PI_4, 4*sqrt(2));
        seeds[4] = Seed(4, 0, 0, 4);
        
    }
    
    double SSPlanner::heuristicCost(const SSState &current, const SSState &goal) const  {
            return sqrt(current.distanceSqTo(goal));
    }
    
    // make array immutable
    std::array<SSState, NO_OF_NEIGHBORS> SSPlanner::neighborNodes(const SSState& current) const    {
        
        std::array<SSState, NO_OF_NEIGHBORS> neighbors ;
        
        // add tentative g_score
        int i = 0;
        
        for (auto& neighbor : neighbors) {
            neighbor = current + seeds[i];
            ++i;
        }
        
        return neighbors;
    }
    
    PathPtr SSPlanner::reconstructPath(const std::unordered_map<SSState, SSState>& came_from, const SSState& current, const SSState& start) const {
	
		SSState temp(current);
        while ( temp != start) {
			
			cv::circle(img, cvPoint(temp.x(), temp.y()), 5, cvScalarAll(255), -1);
			temp = came_from[temp];
			
		}
		
        return nullptr;
    }

    PathPtr SSPlanner::traversablePath(const State& start_, const State& goal_)  const {
        
        const SSState start{start_,0,heuristicCost(start_, goal_)};
        const SSState goal{goal_};
        
        //TODO : make it efficieant mem and cpu copy
        //TODO : can we eleminate closedSet
        std::unordered_set<SSState> closedSet{};
        std::SSPriorityQueue <SSState> openSet = { start };
        
//        std::unordered_set<State> openSet = { start };
        
        //TODO : can we eleminate camefrom
        std::unordered_map<SSState, SSState> cameFrom = {};
        
        //TODO : vector or inbuilt gScore ??
        
//        std::unordered_map<SSState, double> gScore;
//        std::unordered_map<SSState, double> fScore;
        
        
        while (!openSet.empty()) {
            
            const SSState current = openSet.top();
            
            if (current == goal)
                return reconstructPath(cameFrom, current, start);

            openSet.pop();
            
            closedSet.insert(current);
            
            std::array<SSState, NO_OF_NEIGHBORS> neighbpurs;
//            std::for_each(neighbpurs.begin(), neighbpurs.end(), _Function __f)
            
            for (const auto& neighbor : neighbpurs) {
                if (closedSet.find(neighbor) != closedSet.end()) {
                    continue;
                }
                
//                neighbor.updateCost();
                cameFrom[neighbor] = current;
                
				openSet.push(neighbor);
				
//                openSet.updateElement(neighbor, neighbor);
            
            }
           
            
        }
        
        return nullptr;
    }


	void SSPlanner::showPath()	{
		
		cv::imshow("SS_PLANNER", img);
		cv::waitKey(0);
		
	}
}