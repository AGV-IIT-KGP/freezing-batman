//
//  SSPlanner.cpp
//  GlobalPlanner
//
//  Created by Satya Prakash on 28/01/14.
//  Copyright (c) 2014 Satya Prakash. All rights reserved.
//

#include "SSPlanner.hpp"
#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <functional>
#include <array>

#include "SSPriority_queue.hpp"

namespace navigation {
    
    
    PathPtr SSPlanner::traversablePath(const State& start_, const State& goal_) const {
        
        const SSState start{start_,0,heuristicCost(start_, goal_, Heuristic::SS_TEMP)};
        const SSState goal{goal_};
        
//        img = 1;
        cv::circle(img, cvPoint(goal.x(), goal.y()), 5, cvScalarAll(255), -1);

        
        //TODO : make it efficieant mem and cpu copy
        //TODO : can we eleminate closedSet
        std::unordered_set<SSState> closedSet{};
        SS::PriorityQueue <SSState> openSet = { start };
        
        //        std::unordered_set<State> openSet = { start };
        
        //TODO : can we eleminate camefrom
        std::unordered_map<SSState, SSState> cameFrom = {};

        //TODO : can we eleminate camefrom
        std::unordered_set<SSState> openMap = {};
        
        
        //TODO : vector or inbuilt gScore ??
        
        //        std::unordered_map<SSState, double> gScore;
        //        std::unordered_map<SSState, double> fScore;
        
        
        while (!openSet.empty()) {
            
            const SSState current = openSet.top();
            
            openMap.insert(current);
//            std::cout<<"SIze of OpenSet : "<<openSet.size()<<std::endl;
//            std::cout<<"current x : "<<current.x()<<" current y : "<<current.y()<<std::endl;
            
//            cv::circle(img, cvPoint(current.x(), current.y()), 5, cvScalarAll(255), -1);
//            cv::imshow("SS_PLANNER", img);
//            cv::waitKey(0);
            
            if (isCloseTo(current,  goal))  {
                printf("Openlist Size : %lu, came_from size : %lu\n", openSet.size(), cameFrom.size());
                return reconstructPath(cameFrom, current, start);
            }
            
            openSet.pop();
            
            closedSet.insert(current);
            
            std::array<SSState, NO_OF_NEIGHBORS> neighbpurs = neighborNodes(current, goal);
            
            //            std::for_each(neighbpurs.begin(), neighbpurs.end(), _Function __f)
            
            for (const auto& neighbor : neighbpurs) {
                if (closedSet.find(neighbor) != closedSet.end()) {
                    continue;
                }
                if (openMap.find(neighbor) != openMap.end()) {
                    continue;
                }
                openMap.insert(neighbor);

                //                neighbor.updateCost();
                cameFrom[neighbor] = current;
                
				openSet.push(neighbor);
				
                //                openSet.updateElement(neighbor, neighbor);
                
            }
            
            
        }
        
        return nullptr;
    }

    
    SSPlanner::SSPlanner()  {

        img = cv::Mat{cvSize(IMG_WIDTH, IMG_HEIGHT),CV_8UC3,cvScalarAll(0)};
        
        double diagonalMulti = sqrt(2) + 0.01;
        
        seeds[0] = Seed(-seedLength, 0, -M_PI/2, seedLength);
        seeds[1] = Seed(-seedLength, seedLength, -M_PI/4, seedLength * diagonalMulti);
        seeds[2] = Seed(0, seedLength, 0, seedLength);
        seeds[3] = Seed(seedLength, seedLength, M_PI_4, seedLength * diagonalMulti);
        seeds[4] = Seed(seedLength, 0, M_PI_2, seedLength);
        seeds[5] = Seed(-seedLength, -seedLength, M_PI_2, seedLength * diagonalMulti);
        seeds[6] = Seed(0, -seedLength, M_PI_2, seedLength);
        seeds[7] = Seed(seedLength, -seedLength, M_PI_2, seedLength * diagonalMulti);
        
    }
    
    double SSPlanner::heuristicCost(const SSState &current, const SSState &goal, Heuristic heuristic) const  {
        
        int delta_x = std::abs(current.x() - goal.x());
        int delta_y = std::abs(current.y() - goal.y());
        
        switch (heuristic) {
                
            case Heuristic::SS_HEUCLIDIAN:
                return current.distanceTo(goal);
                break;
                
            case Heuristic::SS_HMANHATTAN:
                return current.manhattanDistanceTo(goal);
                break;
                
            case Heuristic::SS_TEMP:
                return delta_x > delta_y ? delta_y*sqrt(2) + (delta_x - delta_y) : delta_x*sqrt(2) + (delta_y - delta_x);
                break;
                
            default:
                break;
        }
        
    }
    
    
    bool SSPlanner::isCloseTo(const SSState& current,const navigation::SSState &goal) const {
        
        return current.distanceTo(goal) < seedLength*10;
    }
    // make array immutable

    std::array<SSState, NO_OF_NEIGHBORS> SSPlanner::neighborNodes(const SSState& current, const SSState& goal) const    {
        
        std::array<SSState, NO_OF_NEIGHBORS> neighbors ;
        
        // add tentative g_score
        int i = 0;
        
        for (auto& neighbor : neighbors) {
            neighbor = current + seeds[i];
            neighbor.hCost((int)heuristicCost(neighbor, goal, Heuristic::SS_TEMP));
            neighbor.updateTotalCost();
            ++i;
            
//            std::cout<<"neighbor x : "<<neighbor.x()<<" neighbor y : "<<neighbor.y()<<" cost : "<<neighbor.totalCost()<<std::endl;

        }
        
        
        
        
        return neighbors;
    }
    
    PathPtr SSPlanner::reconstructPath(std::unordered_map<SSState, SSState>& came_from, const SSState& current, const SSState& start) const{
	
		SSState temp(current);
        
        std::vector<SSState> segments= { temp };
        
        while ( temp != start) {
			
//            std::cout<<"x : "<<temp.x()<<" y : "<<temp.y()<<std::endl;
//			cv::circle(img, cvPoint(temp.x(), temp.y()), 5, cvScalarAll(255), -1);
			temp = came_from[temp];
            
            segments.push_back(temp);
		}
        
        PathSegmentPtr pathSegments( new SSPathSegment(0,segments));
        PathPtr path (new SSPath(0, {pathSegments}));
		
        return path;
    }


	void SSPlanner::showPath(PathPtr path) const    {
        
        img = 1;
        
        for (auto segments : path->segments()) {
            for ( auto state : segments->states() ) {
                cv::circle(img, cvPoint(state.x(), state.y()), 5, cvScalarAll(255), -1);
            }
        }
		cv::imshow("SS_PLANNER", img);
		cv::waitKey(0);
		
	}
}