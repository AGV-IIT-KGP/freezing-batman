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
        int vmax = 70;
        int max_iterations = 10000;
        int min_radius = 70;
        int num_seeds;
        int return_status;
        double x, y, z;

        node_handle.getParam("local_planner/vmax", vmax);
        node_handle.getParam("local_planner/max_iterations_load_given_seeds", max_iterations);
        node_handle.getParam("local_planner/min_rad", min_radius);

        //work TODO change to c++
        FILE *seeds_file = fopen(seeds_file_name.c_str(), "r");

        if (!seeds_file) {
            std::cout << "load in opening seed file : " << seeds_file_name << std::endl;
        }
        return_status = fscanf(seeds_file, "%d\n", &num_seeds);
        if (return_status == 0) {
            //ROS_ERROR("[PLANNER] Incorrect seed file format");
            //Planner::finBot();
            exit(1);
        }

        for (int i = 0; i < num_seeds; i++) {
            Seed seed;
            return_status = fscanf(seeds_file, "%lf %lf %lf %lf %lf\n", &seed.velocityRatio, &x, &y, &z, &seed.obstacleCostOfSeed);
            if (return_status == 0) {
                //                ROS_ERROR("[PLANNER] Incorrect seed file format");
                //                Planner::finBot();
                exit(1);
            }

            seed.leftVelocity = vmax * seed.velocityRatio / (1 + seed.velocityRatio);
            seed.rightVelocity = vmax / (1 + seed.velocityRatio);

            seed.final_state = State((int) x, (int) y, z, 0);

            int num_seed_points;
            return_status = fscanf(seeds_file, "%d\n", &num_seed_points);
            if (return_status == 0) {
                std::cout << "[PLANNER] Incorrect seed file format";
                exit(1);
            }

            for (int j = 0; j < num_seed_points; j++) {
                double temp_x, temp_y;
                return_status = fscanf(seeds_file, "%lf %lf \n", &temp_x, &temp_y);
                State point((int) temp_x, (int) temp_y, 0, 0);

                if (return_status == 0) {
                    //ROS_ERROR("[PLANNER] Incorrect seed file format");
                    exit(1);
                }

                seed.intermediatePoints.insert(seed.intermediatePoints.begin(), point);
            }
            givenSeeds.push_back(seed);
        }
    }

    void quickReflex::loadGivenSeeds(const State& start, const State& goal) {
        int VMAX = 70;
        int numberOfSeeds;
        int return_status;
        double x, y, z;
        int DT_CONSTANT = 2;
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
            Seed seed;
            double cost = 0;
            double cost_DT = 0;
            return_status = fscanf(textFileOFSeeds, "%lf %lf %lf %lf\n", &seed.velocityRatio, &x, &y, &z);
            if (return_status == 0) {
                // ROS_ERROR("[PLANNER] Incorrect seed file format");
                exit(1);
            }

            if (y > 0) {
                seed.leftVelocity = VMAX * seed.velocityRatio / (1 + seed.velocityRatio);
                seed.rightVelocity = VMAX / (1 + seed.velocityRatio);
            } else {
                seed.leftVelocity = -VMAX * seed.velocityRatio / (1 + seed.velocityRatio);
                seed.rightVelocity = -VMAX / (1 + seed.velocityRatio);
            }
            seed.final_state = State((int) x, (int) y, z, 0);

            int n_seed_points;
            return_status = fscanf(textFileOFSeeds, "%d\n", &n_seed_points);
            if (return_status == 0) {
                std::cout << "[PLANNER] Incorrect seed file format";
                exit(1);
            }

            for (int j = 0; j < n_seed_points; j++) {
                double tempXvalue, tempYvalue;
                return_status = fscanf(textFileOFSeeds, "%lf %lf\n", &tempXvalue, &tempYvalue);
                State point((int) tempXvalue, (int) tempYvalue, 0, 0);
                State point2((int) start.x() + tempXvalue, (int) start.y() + tempYvalue, 0, 0);

                if (return_status == 0) {
                    //ROS_ERROR("[PLANNER] Incorrect seed file format");
                    exit(1);
                }

                cost += point2.distanceTo(goal);
                cost_DT += fusion_map.at<uchar>(fusion_map.rows - point2.y() - 1, point2.x());
                seed.intermediatePoints.insert(seed.intermediatePoints.begin(), point);
            }
            seed.targetCostOfSeed = (cost / n_seed_points) / 900 + fabs(atan2f((goal.y() - start.y()), (goal.x() - start.x())) - atan2f(y, x)) / M_PI;
            seed.obstacleCostOfSeed = ((cost_DT / n_seed_points) / 255);
            givenSeeds.push_back(seed);
        }

        fclose(textFileOFSeeds);
    }
}