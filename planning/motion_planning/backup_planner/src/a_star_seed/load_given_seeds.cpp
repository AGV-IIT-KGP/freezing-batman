//
//  a_star_seed.cpp
//  AStarSeed
//
//  Created by Satya Prakash on 06/09/13.
//  Copyright (c) 2013 Satya Prakash. All rights reserved.
//


#include "a_star_seed/a_star_seed.hpp"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
namespace navigation {
    void AStarSeed::loadGivenSeeds() {
        const int VMAX = 70;
        const int MAX_ITER = 10000;
        const int MIN_RAD = 70;
        int numberOfSeeds;
        int return_status;
        double x, y, z;
        
        //work TODO change to c++
        FILE *textFileOFSeeds = fopen(SEEDS_FILE.c_str(), "r");
        
        if (!textFileOFSeeds) {
            std::cout<<"load in opening seed file : "<<SEEDS_FILE<<std::endl;
        }
        return_status = fscanf(textFileOFSeeds, "%d\n", &numberOfSeeds);
        if (return_status == 0) {
            //ROS_ERROR("[PLANNER] Incorrect seed file format");
            //Planner::finBot();
            exit(1);
        }
        
        for (int i = 0; i < numberOfSeeds; i++) {
            Seed s;
            return_status = fscanf(textFileOFSeeds, "%lf %lf %lf %lf %lf\n", &s.velocityRatio, &x, &y, &z, &s.costOfseed);
            if (return_status == 0) {
                //                ROS_ERROR("[PLANNER] Incorrect seed file format");
                //                Planner::finBot();
                exit(1);
            }
            
            s.leftVelocity = VMAX * s.velocityRatio / (1 + s.velocityRatio);
            s.rightVelocity = VMAX / (1 + s.velocityRatio);
            
            s.finalState = State((int)x,(int)y,z,0);

            int n_seed_points;
            return_status = fscanf(textFileOFSeeds, "%d\n", &n_seed_points);
            if (return_status == 0) {
                std::cout<<"[PLANNER] Incorrect seed file format";
                exit(1);
            }
            
            for (int j = 0; j < n_seed_points; j++) {
                double tempXvalue,tempYvalue;
                return_status = fscanf(textFileOFSeeds, "%lf %lf \n", &tempXvalue, &tempYvalue);
                State point((int)tempXvalue, (int)tempYvalue,0 ,0);

                if (return_status == 0) {
                    //ROS_ERROR("[PLANNER] Incorrect seed file format");
                    exit(1);
                }
                
                s.intermediatePoints.insert(s.intermediatePoints.begin(), point);
            }
            givenSeeds.push_back(s);
        }
    }
}
