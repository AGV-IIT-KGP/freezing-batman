#include "quick_reflex/quick_reflex.hpp"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

namespace navigation {

    void quickReflex:: loadGivenSeeds(const State& start, const State& goal) {
        const int VMAX = 70;
        int numberOfSeeds;
        int return_status;
        double x, y, z;
        
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
            double cost=0;
            return_status = fscanf(textFileOFSeeds, "%lf %lf %lf %lf\n", &s.velocityRatio, &x, &y, &z);
            if (return_status == 0) {
                //                ROS_ERROR("[PLANNER] Incorrect seed file format");
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
                
                cost += point.distanceTo(goal) + fusionMap.at<uchar>(fusionMap.rows - intermediateYcordinate -1, intermediateXcordinate);              
                s.intermediatePoints.insert(s.intermediatePoints.begin(), point);
            }
            s.costOfseed = (cost / n_seed_points);
            givenSeeds.push_back(s);
        }
    }
}
