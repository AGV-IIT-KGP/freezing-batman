//
//  AStarGrid.cpp
//  GlobalPlanner
//
//  Created by Satya Prakash on 06/09/13.
//  Copyright (c) 2013 Satya Prakash. All rights reserved.
//

#include "a_star_grid/AStarGrid.hpp"
#include <vector>

namespace navigation {
    ///Triplet locationOfBot,Triplet targetLocation_
    
    void SdcPlanner::plannerWithSimpleAstar(const State& start_, const State& target_)    {
        
        sPriorityQueue<GridState> openSet;
        GridState start(start_,0,0);
        GridState target(target_,0,0);
        start.updateTotalCost(target);
        openSet.push(start);
        
        while(!openSet.empty())
        {
            const GridState currentState=openSet.top();
            openSet.pop();
            auto currentX=currentState.x();
            auto currentY=currentState.y();
            openNodes.at<uchar>(currentX, currentY)=0;
            closedNodes.at<uchar>(currentX, currentY)=1;
            
            if(currentState == target)
            {
                reconstructPath(currentState, start, target);
                return ;
            }
            const auto neighbors = neighborNodes(currentState,target);
            populateOpenList(openSet, neighbors);
        }
        
        std::cerr<<"NO PATH FOUND"<<std::endl;
        return ; // no route found
        
    }
    int getPt( int n1 , int n2 , float perc )
    {
        int diff = n2 - n1;
        return n1 + ( diff * perc );
    }
    SdcPlanner::SdcPlanner()
    {
        mapWithObstacles =cv::Mat::zeros(IMAGE_WIDTH, IMAGE_HEIGHT, CV_8UC1);
        obstacleMap =cv::Mat::zeros(MAP_WIDTH, MAP_HEIGHT, CV_8SC1);
        closedNodes =cv::Mat::zeros(MAP_WIDTH, MAP_HEIGHT, CV_8SC1);
        openNodes =cv::Mat::zeros(MAP_WIDTH, MAP_HEIGHT, CV_16UC1);
        directionMap =cv::Mat::zeros(MAP_WIDTH, MAP_HEIGHT, CV_8SC1);
    }
    
    void SdcPlanner::addObstacles()
    {
        
        for(int height_=MAP_HEIGHT/8;height_<MAP_HEIGHT*7/8;height_++)
        {
            obstacleMap.at<uchar>(height_,MAP_WIDTH/2)=1;
        }
        for(int width_=MAP_HEIGHT/8;width_<MAP_WIDTH*7/8;width_++)
        {
            obstacleMap.at<uchar>(MAP_HEIGHT/2,width_)=1;
        }
        
    }
//    void SdcPlanner::addStartAndEndPoints()
//    {
//        srand((unsigned int)time(NULL));
//        int botLocationX, botLocationY, targetLocationX, targetLocationY;
//        switch(rand()%8)
//        {
//            case 0: botLocationX=0;botLocationY=0;targetLocationX=MAP_WIDTH-1;targetLocationY=MAP_HEIGHT-1; break;
//            case 1: botLocationX=0;botLocationY=MAP_HEIGHT-1;targetLocationX=MAP_WIDTH-1;targetLocationY=0; break;
//            case 2: botLocationX=MAP_WIDTH/2-1;botLocationY=MAP_HEIGHT/2-1;targetLocationX=MAP_WIDTH/2+1;targetLocationY=MAP_HEIGHT/2+1; break;
//            case 3: botLocationX=MAP_WIDTH/2-1;botLocationY=MAP_HEIGHT/2+1;targetLocationX=MAP_WIDTH/2+1;targetLocationY=MAP_HEIGHT/2-1; break;
//            case 4: botLocationX=MAP_WIDTH/2-1;botLocationY=0;targetLocationX=MAP_WIDTH/2+1;targetLocationY=MAP_HEIGHT-1; break;
//            case 5: botLocationX=MAP_WIDTH/2+1;botLocationY=MAP_HEIGHT-1;targetLocationX=MAP_WIDTH/2-1;targetLocationY=0; break;
//            case 6: botLocationX=0;botLocationY=MAP_HEIGHT/2-1;targetLocationX=MAP_WIDTH-1;targetLocationY=MAP_HEIGHT/2+1; break;
//            case 7: botLocationX=MAP_WIDTH-1;botLocationY=MAP_HEIGHT/2+1;targetLocationX=0;targetLocationY=MAP_HEIGHT/2-1; break;
//        }
//        
//        locationOfBot.x(botLocationX);
//        locationOfBot.y(botLocationY);
//        locationOfBot.z(0);
//        targetLocation.x(targetLocationX);
//        targetLocation.y(targetLocationY);
//        targetLocation.z(0);
//        
//        
//        
//        std::cout<<"Map Size (X,Y): "<<MAP_WIDTH<<","<<MAP_HEIGHT<<std::endl;
//        std::cout<<"Start: "<<botLocationX<<","<<botLocationY<<std::endl;
//        std::cout<<"Finish: "<<targetLocationX<<","<<targetLocationY<<std::endl;
//    }
    void SdcPlanner::tester()
    {
        addObstacles();
        srand((unsigned int) time(nullptr));
        State current { rand()%MAP_WIDTH, rand()%MAP_HEIGHT, 0, 0 }, target { rand()%MAP_WIDTH, rand()%MAP_HEIGHT, 0, 0 };
        
        std::cout<<"Start Point : "<<current.x()<<" "<<current.y()<<" ";
        std::cout<<"Target Point : "<<target.x()<<" "<<target.y()<<" ";

        
        clock_t start = clock();
        plannerWithSimpleAstar(current, target);
        if(pathToTarget=="") std::cout<<"An empty route generated!"<<std::endl;
        clock_t end = clock();
        double time_elapsed = double(end - start);
        std::cout<<"Time to calculate the route (ms): "<<time_elapsed<<std::endl;
        
        if(pathToTarget.length()>0)
        {
            int jump; char DirectionToNextState;
            int intermediateXcordinte=current.x();
            int intermediateYcordinte=target.y();
            
            obstacleMap.at<uchar>(intermediateXcordinte,intermediateYcordinte)=2;
            
            for(int i=0;i<pathToTarget.length();i++)
            {
                DirectionToNextState =pathToTarget.at(i);
                jump=atoi(&DirectionToNextState);
                intermediateXcordinte=intermediateXcordinte+directionOfGrids[jump].x();
                intermediateYcordinte=intermediateYcordinte+directionOfGrids[jump].y();
                obstacleMap.at<uchar>(intermediateXcordinte,intermediateYcordinte)=3;
            }
            obstacleMap.at<uchar>(intermediateXcordinte,intermediateYcordinte)=4;
            for(intermediateYcordinte=0;intermediateYcordinte<MAP_HEIGHT;intermediateYcordinte++)
            {
                for( intermediateXcordinte=0;intermediateXcordinte<MAP_WIDTH;intermediateXcordinte++)
                    
                    if(obstacleMap.at<uchar>(intermediateXcordinte,intermediateYcordinte)==1)
                    {
                        
                        cv::rectangle(mapWithObstacles, cvPoint(intermediateXcordinte*10, intermediateYcordinte*10), cvPoint(intermediateXcordinte*10+10, intermediateYcordinte*10+10), cvScalar(128),-1);
                    }
                    else if(obstacleMap.at<uchar>(intermediateXcordinte,intermediateYcordinte) == 2){
                        cv::rectangle(mapWithObstacles, cvPoint(intermediateXcordinte*10, intermediateYcordinte*10), cvPoint(intermediateXcordinte*10+10, intermediateYcordinte*10+10), cvScalar(255));
                    }
                    else if(obstacleMap.at<uchar>(intermediateXcordinte,intermediateYcordinte) == 3){
                        //                        cv::rectangle(mapWithObstacles, cvPoint(intermediateXcordinte*10, intermediateYcordinte*10), cvPoint(intermediateXcordinte*10+10, intermediateYcordinte*10+10), cvScalar(255),-1);
                    }
                    else if(obstacleMap.at<uchar>(intermediateXcordinte,intermediateYcordinte) == 4){
                        cv::rectangle(mapWithObstacles, cvPoint(intermediateXcordinte*10, intermediateYcordinte*10), cvPoint(intermediateXcordinte*10+10, intermediateYcordinte*10+10), cvScalar(255));
                    }
            }
        }
        cv::imshow("Planner_mapSS", mapWithObstacles);
        cvWaitKey(0);
        return;
    }
    
    void SdcPlanner::reconstructPath(const GridState& current, const GridState& start, const GridState& target)
    {
        int jump;
        char pathCordinate;
        int xValueP,yValueP;
        pathToTarget ="";
        
        xValueP=current.x();
        yValueP=current.y();
        int x1=0,x2=0,y1=0,y2=0,y3=0,x3=0,x4=0,y4=0,x5=0,y5=0;
        int xa,xb,ya,yb,x_,y_;
        
        while(!(xValueP==start.x() && yValueP==start.y()))
        {
            x1=x4;y1=y4;
            x4=x2;y4=y2;
            x2=x5;y2=y5;
            x5=x3;y5=y3;
            x3=xValueP*10;
            y3=yValueP*10;
            
            for( double i = 0 ; i < 1 ; i += 0.01 )
            {
                // The Green Line
                xa = getPt( x1 , x2 , i );
                ya = getPt( y1 , y2 , i );
                xb = getPt( x2 , x3 , i );
                yb = getPt( y2 , y3 , i );
                // The Black Dot
                x_ = getPt( xa , xb , i );
                y_ = getPt( ya , yb , i );
                
                if (x1!=0||y1!=0) {
                    cv::circle(mapWithObstacles, cvPoint(x_, y_), 1, cvScalar(255));
                }
            }
            jump=directionMap.at<uchar>(xValueP,yValueP);
            pathCordinate='0'+(jump+NUMBER_OF_DIRECTIONS/2)%NUMBER_OF_DIRECTIONS;
            pathToTarget=pathCordinate+pathToTarget;
            xValueP+=directionOfGrids[jump].x();
            yValueP+=directionOfGrids[jump].y();
        }
    }
    
    const std::vector<GridState> SdcPlanner::neighborNodes(const GridState& current,const GridState& target)
    {
        //TODO remove i
        int i=0;
        
        std::vector<GridState> neighbors;

        auto xValueP = current.x();
        auto yValueP = current.y();
        for(auto& direction : directionOfGrids)
        {
            xValueNeighbor=xValueP+direction.x();
            yValueNeighbor=yValueP+direction.y();
            if(!(xValueNeighbor<0 || xValueNeighbor>MAP_WIDTH-1 || yValueNeighbor<0 || yValueNeighbor>MAP_HEIGHT-1 || obstacleMap.at<uchar>(xValueNeighbor,yValueNeighbor)==1
                 || closedNodes.at<uchar>(xValueNeighbor,yValueNeighbor)==1))
            {
                
                GridState neighbor(xValueNeighbor, yValueNeighbor, 0, 0, current.getGcostAlongFollowedPath(), current.getApproxCostOfPathToTarget());
                neighbor.setDirectionOfParent(i);
                neighbor.updateGcostOfneighbour(i);
                neighbor.updateTotalCost(target);
                neighbors.push_back(neighbor);
            }
            i++;
        }

        return neighbors;
    }
    
    void SdcPlanner::populateOpenList(sPriorityQueue<GridState>& openSet, const std::vector<GridState>& neighbors)  {
        for (auto& neighbor : neighbors) {
            
            xValueNeighbor=neighbor.x();
            yValueNeighbor=neighbor.y();
            // TODO change opnNodes data structure to 32 bit image not 8 bit
            if(openNodes.at<uchar>(xValueNeighbor,yValueNeighbor)==0)
            {
                openNodes.at<uchar>(xValueNeighbor,yValueNeighbor)=neighbor.getApproxCostOfPathToTarget();
                openSet.push(neighbor);
                directionMap.at<uchar>(xValueNeighbor,yValueNeighbor)=(neighbor.getDirectionFromParent()+NUMBER_OF_DIRECTIONS/2)%NUMBER_OF_DIRECTIONS;
            }
            else if(openNodes.at<uchar>(xValueNeighbor,yValueNeighbor)>neighbor.getApproxCostOfPathToTarget())
            {
                openNodes.at<uchar>(xValueNeighbor,yValueNeighbor)=neighbor.getApproxCostOfPathToTarget();
                directionMap.at<uchar>(xValueNeighbor,yValueNeighbor)=(neighbor.getDirectionFromParent()+NUMBER_OF_DIRECTIONS/2)%NUMBER_OF_DIRECTIONS;
                
                openSet.removeElement(neighbor);
                openSet.push(neighbor);
            }
        }
    }
}

