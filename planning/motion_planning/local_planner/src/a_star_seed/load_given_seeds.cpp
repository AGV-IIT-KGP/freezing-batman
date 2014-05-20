//
//  a_star_seed.cpp
//  AStarSeed
//
//  Created by Satya Prakash on 06/09/13.
//  Copyright (c) 2013 Satya Prakash. All rights reserved.
//

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <a_star_seed/a_star_seed.hpp>

namespace navigation {

    void AStarSeed::loadGivenSeeds() {
        int VMAX;
        int MAX_ITER;
        int MIN_RAD;
        int numberOfSeeds;
        int return_status;
        double x, y, z;

        node_handle.getParam("local_planner/vmax", VMAX);
        node_handle.getParam("local_planner/max_iterations_load_given_seeds", MAX_ITER);
        node_handle.getParam("local_planner/min_rad", MIN_RAD);

        //work TODO change to c++
        FILE *textFileOFSeeds = fopen(SEEDS_FILE.c_str(), "r");

        if (!textFileOFSeeds) {
            std::cout << "load in opening seed file : " << SEEDS_FILE << std::endl;
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

            s.final_state = State((int) x, (int) y, z, 0);

            int n_seed_points;
            return_status = fscanf(textFileOFSeeds, "%d\n", &n_seed_points);
            if (return_status == 0) {
                std::cout << "[PLANNER] Incorrect seed file format";
                exit(1);
            }

            for (int j = 0; j < n_seed_points; j++) {
                double tempXvalue, tempYvalue;
                return_status = fscanf(textFileOFSeeds, "%lf %lf \n", &tempXvalue, &tempYvalue);
                State point((int) tempXvalue, (int) tempYvalue, 0, 0);

                if (return_status == 0) {
                    //ROS_ERROR("[PLANNER] Incorrect seed file format");
                    exit(1);
                }

                s.intermediatePoints.insert(s.intermediatePoints.begin(), point);
            }
            givenSeeds.push_back(s);
        }
    }

    void quickReflex::loadGivenSeeds(const State& start, const State& goal) {
        int VMAX;
        int numberOfSeeds;
        int return_status;
        double x, y, z;
        int DT_CONSTANT;
        node_handle.getParam("local_planner/distance_transform_constant", DT_CONSTANT);
        node_handle.getParam("local_planner/vmax", VMAX);
        FILE *textFileOFSeeds = fopen(seeds_file.c_str(), "r");

        if (!textFileOFSeeds) {
            std::cout << "load in opening seed file : " << seeds_file << std::endl;
        }

        return_status = fscanf(textFileOFSeeds, "%d\n", &numberOfSeeds);

        if (return_status == 0) {
            //ROS_ERROR("[PLANNER] Incorrect seed file format");
            //Planner::finBot();
            exit(1);
        }

        for (int i = 0; i < numberOfSeeds; i++) {
            Seed s;
            double cost = 0;
            return_status = fscanf(textFileOFSeeds, "%lf %lf %lf %lf\n", &s.velocityRatio, &x, &y, &z);
            if (return_status == 0) {
                //                ROS_ERROR("[PLANNER] Incorrect seed file format");
                exit(1);
            }

            s.leftVelocity = VMAX * s.velocityRatio / (1 + s.velocityRatio);
            s.rightVelocity = VMAX / (1 + s.velocityRatio);

            s.final_state = State((int) x, (int) y, z, 0);

            int n_seed_points;
            return_status = fscanf(textFileOFSeeds, "%d\n", &n_seed_points);
            if (return_status == 0) {
                //std::cout << "[PLANNER] Incorrect seed file format";
                exit(1);
            }

            for (int j = 0; j < n_seed_points; j++) {
                double tempXvalue, tempYvalue;
                return_status = fscanf(textFileOFSeeds, "%lf %lf \n", &tempXvalue, &tempYvalue);
                State point((int) (start.x()+tempXvalue), (int) (start.y()+tempYvalue), 0, 0);

                if (return_status == 0) {
                    //ROS_ERROR("[PLANNER] Incorrect seed file format");
                    exit(1);
                }

                cost += point.distanceTo(goal);// + DT_CONSTANT * fusion_map.at<uchar>(fusion_map.rows - tempYvalue - 1, tempXvalue);
                State point2((int) tempXvalue, (int) tempYvalue, 0, 0);
                s.intermediatePoints.insert(s.intermediatePoints.begin(), point2);
            }
            if(start.x() == goal.x()){
                if(x!=0)
                    s.costOfseed = (cost / n_seed_points)/900 + fabs(atanf(y/x) - M_PI/2)/M_PI;
                else
                    s.costOfseed = (cost / n_seed_points)/900;
            }
            else{
                if(x!=0)
                    s.costOfseed = (cost / n_seed_points)/900 + fabs(atanf(y/x) - atanf((goal.y()-start.y())/(goal.x()-start.x())))/M_PI;
                else
                    s.costOfseed = (cost / n_seed_points)/900 + fabs(M_PI/2 - atanf((goal.y()-start.y())/(goal.x()-start.x())))/M_PI;
            }
            givenSeeds.push_back(s);
        }

        fclose(textFileOFSeeds);
    }
}
