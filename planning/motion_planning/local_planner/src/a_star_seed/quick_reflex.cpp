#include <a_star_seed/a_star_seed.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>

const int INFINITE = 99999;

namespace navigation {

    std::pair<std::vector<State>, Seed> quickReflex::findPathToTarget(const cv::Mat& img, const State& start, const State& goal, int& status) {
        fusion_map = img;
        Seed resultSeed;
        float pi = 3.14159;
        int count;
        count = 0;
        int thresh = 250;
        distanceTransform();
        angle_limit = 45;
        curvature_max = .001; // min rad of curvature = 1m = 100cm
        dist_bw_wheels = 80;
        first_half_intensity = 0;
        second_half_intnesity = 0;
        flag = true;
        bool npf = false;
        min_cost = INFINITE;
        min_cost_weight = INFINITE;
        num_of_vectors = 120;
        num_of_points = 20;
        vector_radius = 200;
        vmax = 70;
        vfs = true;
        bool vel_profile = false;

        node_handle.getParam("/local_planner/angle_limit", angle_limit);
        node_handle.getParam("/local_planner/curvature_max", curvature_max);
        node_handle.getParam("/local_planner/dist_bw_wheels", dist_bw_wheels);
        node_handle.getParam("/local_planner/min_cost", min_cost);
        node_handle.getParam("/local_planner/num_of_vectors", num_of_vectors);
        node_handle.getParam("/local_planner/num_of_points", num_of_points);
        node_handle.getParam("/local_planner/vector_radius", vector_radius);
        node_handle.getParam("/local_planner/vmax", vmax);
        node_handle.getParam("/local_planner/vfs", vfs);

        if (vfs) {
            cv::namedWindow("view", CV_WINDOW_NORMAL);
            cv::imshow("view", fusion_map);
            cv::waitKey(10);
            if (flag) {
                for (iterator1 = 0; iterator1 < num_of_vectors; iterator1++) {
                    seed_is_walkable = true;
                    cost = 0;
                    theta = pi / 2 - pi * angle_limit / 180 + 2 * pi * angle_limit * iterator1 / (num_of_vectors * 180);
                    for (iterator2 = 1; iterator2 <= num_of_points; iterator2++) {
                        intensity = fusion_map.at<uchar>
                                (fusion_map.rows - (start.y() + vector_radius * sin(theta) * iterator2 / num_of_points) + 1,
                                 start.x() + vector_radius * cos(theta) * iterator2 / num_of_points);
                        seed_weight = intensity * (num_of_points - iterator2);
                        if (intensity == 255) {
                            seed_is_walkable = false;
                            break;
                        } else {
                            cost += intensity;
                            weight += seed_weight;
                        }
                    }
                    if (seed_is_walkable) {
                        if (min_cost > cost) {
                            min_cost = cost;
                            min_cost_angle = theta * 180 / pi;
                        } else if (cost == min_cost && weight < min_cost_weight) {
                            min_cost_weight = weight;
                            min_cost_angle = theta * 180 / pi;
                        }
                    }
                }
                if (min_cost == INFINITE) {
                    status = 0;
                    std::cout << "NO PATH FOUND" << std::endl;
                    if (npf) {
                        for (iterator1 = 0; iterator1 < fusion_map.rows; iterator1++) {
                            for (iterator2 = 0; iterator2 < fusion_map.cols / 2; iterator2++) {
                                if (fusion_map.at<uchar>(iterator1, iterator2) < thresh) {
                                    first_half_intensity++;
                                }
                            }

                            for (iterator2 = fusion_map.cols / 2; iterator2 < fusion_map.cols; iterator2++) {
                                if (fusion_map.at<uchar>(iterator1, iterator2) < thresh) {
                                    second_half_intnesity++;
                                }
                            }
                        }

                        if (second_half_intnesity > first_half_intensity) {
                            resultSeed.leftVelocity = -vmax / 2.;
                            resultSeed.rightVelocity = 0;
                        } else {
                            resultSeed.rightVelocity = -vmax / 2.;
                            resultSeed.leftVelocity = 0;
                        }
                    }
                    return std::make_pair(std::vector<State>(), resultSeed);
                }
            } else {
                std::vector<int> min_angles;

                for (iterator1 = 0; iterator1 < num_of_vectors; iterator1++) {
                    min_angles.push_back(iterator1);
                }
                min_cost_angle = 90;
                for (iterator2 = 1; iterator2 <= num_of_points; iterator2 += 2) {
                    double min_intensity = 255;
                    int repeats = 0;
                    for (int angle_id = 0; angle_id < min_angles.size(); angle_id++) {
                        if (min_angles[angle_id] == -1) {
                            continue;
                        }

                        double theta = pi / 2 - pi * angle_limit / 180 + 2 * pi * angle_limit * angle_id / (num_of_vectors * 180);

                        intensity = fusion_map.at<uchar>
                                (fusion_map.rows - (start.y() + vector_radius * sin(theta) * iterator2 / num_of_points) + 1,
                                 start.x() + vector_radius * cos(theta) * iterator2 / num_of_points);

                        if (intensity == 255) {
                            continue;
                        }
                        if (intensity < min_intensity) {
                            min_intensity = intensity;
                            min_cost_angle = theta * 180 / pi;
                        }
                    }

                    //std::cout << "min_intensity: " << min_intensity << std::endl;

                    if (min_intensity == 255) {
                        // No Path Found
                        status = 0;
                        std::cout << "NO PATH FOUND" << std::endl;
                        if (npf) {
                            for (iterator1 = 0; iterator1 < fusion_map.rows; iterator1++) {
                                for (iterator2 = 0; iterator2 < fusion_map.cols / 2; iterator2++) {
                                    if (fusion_map.at<uchar>(iterator1, iterator2) < thresh) {
                                        first_half_intensity++;
                                    }
                                }

                                for (iterator2 = fusion_map.cols / 2; iterator2 < fusion_map.cols; iterator2++) {
                                    if (fusion_map.at<uchar>(iterator1, iterator2) < thresh) {
                                        second_half_intnesity++;
                                    }
                                }
                            }

                            if (second_half_intnesity > first_half_intensity) {
                                resultSeed.leftVelocity = -vmax / 2.;
                                resultSeed.rightVelocity = 0;
                            } else {
                                resultSeed.rightVelocity = -vmax / 2.;
                                resultSeed.leftVelocity = 0;
                            }
                        }
                        return std::make_pair(std::vector<State>(), resultSeed);
                    }

                    //std::cout << "Intensities & Angles:" << std::endl;
                    for (int angle_id = 0; angle_id < min_angles.size(); angle_id++) {
                        theta = pi / 2 - pi * angle_limit / 180 + 2 * pi * angle_limit * angle_id / (num_of_vectors * 180);
                        double x, y;
                        intensity = fusion_map.at<uchar>
                                (x = fusion_map.rows - (start.y() + vector_radius * sin(theta) * iterator2 / num_of_points) + 1,
                                 y = start.x() + vector_radius * cos(theta) * iterator2 / num_of_points);
                        //std::cout << "(" << x << ", " << y << ") " << intensity << ", " << 180 * theta / pi << std::endl;
                        if (intensity != min_intensity) {
                            min_angles[angle_id] = -1;
                        } else {
                            repeats++;
                        }
                    }
                    //std::cout << std::endl;
                    //getchar();

                    //std::cout << "repeats: " << repeats << std::endl;
                    //std::cout << "min_cost_angle: " << min_cost_angle << std::endl;
                    if (repeats == 0) {
                        break;
                    }
                }
            }

            std::cout << "min_cost_angle: " << min_cost_angle << std::endl;
            //min_cost_angle = 180 - min_cost_angle;
            resultSeed.final_state.setcurvature(curvature_max * (90. - min_cost_angle) / angle_limit);
            resultSeed.velocityRatio = (1. - dist_bw_wheels * resultSeed.final_state.curvature()) / (1. + dist_bw_wheels * resultSeed.final_state.curvature());

            double vavg;
            if (vel_profile) {
              vavg = vmax * (.5 + (.1 - .5) * resultSeed.final_state.curvature() / curvature_max);
            } else {
              vavg = vmax / 2.;
            }
            
            resultSeed.rightVelocity = 2 * resultSeed.velocityRatio * vavg / (1 + resultSeed.velocityRatio);
            resultSeed.leftVelocity = 2 * vavg / (1 + resultSeed.velocityRatio);
            State point((int) start.x(), (int) start.y(), 0, 0);
            resultSeed.intermediatePoints.insert(resultSeed.intermediatePoints.begin(), point);
            status = 999;
            return std::make_pair(resultSeed.intermediatePoints, resultSeed);
        } else {
            givenSeeds.clear();
            loadGivenSeeds(start, goal);
            if (start.isCloseTo(goal)) {
                status = 1;
                return std::make_pair(std::vector<State>(), Seed());
            } else if (isOnTheObstacle(start)) {
                printf("Bot is on the Obstacle Map\n");
                return std::make_pair(std::vector<State>(), Seed());
            } else if (isOnTheObstacle(goal)) {
                printf("Target is on the Obstacle Map\n");
                return std::make_pair(std::vector<State>(), Seed());
            }

            // TODO: Either remove INFINITE or handle it properly
            Seed* resultSeed = NULL;
            float minCost = INFINITE;
            std::vector<Seed> neighbours = neighborNodesWithSeeds(start, goal);
            for (int i = 0; i < neighbours.size(); ++i) {
                if (neighbours[i].obstacleCostOfSeed < minCost) {
                    minCost = neighbours[i].obstacleCostOfSeed;
                    resultSeed = &neighbours[i];
                }
            }

            if (resultSeed == NULL) {
                status = 0;
                return std::make_pair(std::vector<State>(), Seed());
            }
            status = 999;

            return std::make_pair(resultSeed->intermediatePoints, *resultSeed);
        }
    }
}
