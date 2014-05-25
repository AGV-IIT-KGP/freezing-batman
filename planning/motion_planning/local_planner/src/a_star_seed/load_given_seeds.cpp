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
            return_status = fscanf(seeds_file, "%lf %lf %lf %lf %lf\n", &seed.velocityRatio, &x, &y, &z, &seed.costOfseed);
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
        int vmax = 70;
        int num_seeds;
        int return_status;
        double x, y, z;
        int dt_constant =2;
        node_handle.getParam("local_planner/distance_transform_constant", dt_constant);
        node_handle.getParam("local_planner/vmax", vmax);
        FILE *textFileOFSeeds = fopen(seeds_file.c_str(), "r");

        if (!textFileOFSeeds) {
            std::cout << "load in opening seed file : " << seeds_file << std::endl;
        }

        return_status = fscanf(textFileOFSeeds, "%d\n", &num_seeds);

        if (return_status == 0) {
            //ROS_ERROR("[PLANNER] Incorrect seed file format");
            //Planner::finBot();
            exit(1);
        }

        for (int i = 0; i < num_seeds; i++) {
            Seed seed;
            double cost = 0;
            double cost_dt = 0;
            return_status = fscanf(textFileOFSeeds, "%lf %lf %lf %lf\n", &seed.velocityRatio, &x, &y, &z);
            if (return_status == 0) {
                //                ROS_ERROR("[PLANNER] Incorrect seed file format");
                exit(1);
            }

            if (y > 0) {
                seed.leftVelocity = vmax * seed.velocityRatio / (1 + seed.velocityRatio);
                seed.rightVelocity = vmax / (1 + seed.velocityRatio);
            } else {
                seed.leftVelocity = -vmax * seed.velocityRatio / (1 + seed.velocityRatio);
                seed.rightVelocity = -vmax / (1 + seed.velocityRatio);
            }
            seed.final_state = State((int) x, (int) y, z, 0);

            int num_seed_points;
            return_status = fscanf(textFileOFSeeds, "%d\n", &num_seed_points);
            if (return_status == 0) {
                //std::cout << "[PLANNER] Incorrect seed file format";
                exit(1);
            }

            for (int j = 0; j < num_seed_points; j++) {
                double temp_x, temp_y;
                return_status = fscanf(textFileOFSeeds, "%lf %lf \n", &temp_x, &temp_y);
                State point((int) temp_x, (int) temp_y, 0, 0);
                State point2((int) start.x() + temp_x, (int) start.y() + temp_y, 0, 0);

                if (return_status == 0) {
                    //ROS_ERROR("[PLANNER] Incorrect seed file format");
                    exit(1);
                }

                cost += point2.distanceTo(goal);
                cost_dt += fusion_map.at<uchar>(fusion_map.rows - (start.x() + temp_x) - 1, start.y() + temp_y);
                seed.intermediatePoints.insert(seed.intermediatePoints.begin(), point);

                seed.costOfseed = (cost / num_seed_points) / 200 + ((cost_dt / num_seed_points) / 255) / dt_constant + abs(atanf((goal.y() - start.y()) / (goal.x() - start.x())) - atanf(y / x));
                cost += point.distanceTo(goal); // + DT_CONSTANT * fusion_map.at<uchar>(fusion_map.rows - tempYvalue - 1, tempXvalue);
                seed.intermediatePoints.insert(seed.intermediatePoints.begin(), point2);

                if (start.x() == goal.x()) {
                    if (x != 0) {
                        seed.costOfseed = (cost / num_seed_points) / 900 + fabs(atanf(y / x) - M_PI / 2) / M_PI;
                    } else {
                        seed.costOfseed = (cost / num_seed_points) / 900;
                    }
                } else {
                    if (x != 0) {
                        seed.costOfseed = (cost / num_seed_points) / 900 + fabs(atanf(y / x) - atanf((goal.y() - start.y()) / (goal.x() - start.x()))) / M_PI;
                    } else {
                        seed.costOfseed = (cost / num_seed_points) / 900 + fabs(M_PI / 2 - atanf((goal.y() - start.y()) / (goal.x() - start.x()))) / M_PI;
                    }
                }
                givenSeeds.push_back(seed);
                fclose(textFileOFSeeds);
            }
        }
    }
}