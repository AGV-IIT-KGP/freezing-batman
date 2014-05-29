#include <a_star_seed/a_star_seed.hpp>

const int INFINITE = 99999;

namespace navigation {

    std::pair<std::vector<State>, Seed> quickReflex::findPathToTarget(const cv::Mat& img, const State& start, const State& goal, int& status) {
        fusion_map = img;

        distanceTransform();

        angle_limit = 45;
        curvature_max = .001; // min rad of curvature = 1m = 100cm
        dist_bw_wheels = 80;
        min_cost = INFINITE;
        num_of_vectors = 120;
        num_of_points = 20;
        vector_radius = 200;
        vmax = 70;
        vfs = true;
        float pi = 3.14159;
        int count;
        count = 0;
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
            bool no_path_found;
            int intensity;
            bool seed_is_walkable;
            no_path_found = false;

            for (iterator1 = 0; iterator1 < num_of_vectors / 2; iterator1++) {
                cost = 0;
                seed_is_walkable = true;
                for (iterator2 = 1; iterator2 <= num_of_points; iterator2++) {
                    intensity = fusion_map.at<uchar>
                            (start.y() + vector_radius * sin(pi * angle_limit / 180 + pi * iterator1 / num_of_vectors) * iterator2 / num_of_points,
                            start.x() + vector_radius * cos(pi * angle_limit / 180 + pi * iterator1 / num_of_vectors) * iterator2 / num_of_points);
                    if (intensity == 255) {
                        seed_is_walkable = false;
                        break;
                    } else {
                        cost += intensity;
                    }
                }
                if (min_cost > cost && seed_is_walkable) {
                    min_cost = cost;
                    min_cost_angle = angle_limit + 180 * iterator1 / num_of_vectors;
                    count++;
                }
            }
            for (iterator1 = num_of_vectors / 2; iterator1 < num_of_vectors; iterator1++) {
                cost = 0;
                seed_is_walkable = true;

                for (iterator2 = 1; iterator2 < num_of_points; iterator2++) {
                    intensity = fusion_map.at<uchar>
                            (start.y() + vector_radius * sin(-pi * angle_limit / 180 + pi * iterator1 / num_of_vectors) * iterator2 / num_of_points,
                            start.x() + vector_radius * cos(-pi * angle_limit / 180 + pi * iterator1 / num_of_vectors) * iterator2 / num_of_points);

                    if (intensity == 255) {
                        seed_is_walkable = false;
                        break;
                    } else {
                        cost += intensity;
                    }
                }
                if (min_cost > cost && seed_is_walkable) {
                    min_cost = cost;
                    min_cost_angle = -angle_limit + 180 * iterator1 / num_of_vectors;
                    count++;
                }
            }
            std::cout << "min_cost_angle: " << min_cost_angle << std::endl;
            Seed resultSeed;
            //resultSeed->final_state.setx((vector_radius * cos(min_cost_angle/180)));
            //resultSeed->final_state.sety((vector_radius * sin(min_cost_angle/180)));
            resultSeed.final_state.setcurvature(curvature_max * (90. - min_cost_angle) / (90. - angle_limit));
            std::cout << "result curvature " << resultSeed.final_state.curvature() << std::endl;
            //resultSeed->final_state.settheta(90);
            resultSeed.obstacleCostOfSeed = min_cost;
            //            std::cout << 1 + resultSeed.final_state.curvature() << " " << 1 - resultSeed.final_state.curvature() << std::endl;
            //            getchar();
            resultSeed.velocityRatio = (1. - dist_bw_wheels * resultSeed.final_state.curvature()) / (1. + dist_bw_wheels * resultSeed.final_state.curvature());
            double vavg = vmax / 2.;
            resultSeed.rightVelocity = 2 * resultSeed.velocityRatio * vavg / (1 + resultSeed.velocityRatio);
            resultSeed.leftVelocity = 2 * vavg / (1 + resultSeed.velocityRatio);
            State point((int) start.x(), (int) start.y(), 0, 0);

            resultSeed.intermediatePoints.insert(resultSeed.intermediatePoints.begin(), point);

            if (min_cost == INFINITE) {
                status = 0;
                return std::make_pair(std::vector<State>(), Seed());
            }
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
