#include <a_star_seed/a_star_seed.hpp>

const int INFINITE = 99999;

namespace navigation {

    std::pair<std::vector<State>, Seed> quickReflex::findPathToTarget(const cv::Mat& img, const State& start, const State& goal, int& status) {
        fusion_map = img;

        distanceTransform();
        angle_limit = 30;
        curvature_max = 100;
        dist_bw_wheels = 70;
        min_cost = INFINITE;
        num_of_vectors = 120;
        num_of_points = 20;
        vector_radius = 70;
        vmax = 70;
        vfs = true;
        float pi = 3.14159;
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
            for (iterator1 = 0; iterator1 < num_of_vectors / 2; iterator1++) {
                cost = 0;
                for (iterator2 = 1; iterator2 <= num_of_points; iterator2++) {
                    cost += fusion_map.at<uchar>
                            ((int) vector_radius * sin(pi * angle_limit / 180 + pi * iterator1 / num_of_vectors) * iterator2 / num_of_points,
                            (int) vector_radius * cos(pi * angle_limit / 180 + pi * iterator1 / num_of_vectors) * iterator2 / num_of_points);
                }
                if (min_cost > cost) {
                    min_cost = cost;
                    min_cost_angle = angle_limit + 180 * iterator1 / num_of_vectors;
                }
            }
            for (iterator1 = num_of_vectors / 2; iterator1 < num_of_vectors; iterator1++) {
                cost = 0;
                for (iterator2 = 1; iterator2 < num_of_points; iterator2++) {
                    cost += fusion_map.at<uchar>
                            ((int) vector_radius * sin(-pi * angle_limit / 180 + pi * iterator1 / num_of_vectors) * iterator2 / num_of_points,
                            (int) vector_radius * cos(-pi * angle_limit / 180 + pi * iterator1 / num_of_vectors) * iterator2 / num_of_points);
                }
                if (min_cost > cost) {
                    min_cost = cost;
                    min_cost_angle = -angle_limit + 180 * iterator1 / num_of_vectors;
                }
            }
            Seed* resultSeed = NULL;
            resultSeed->final_state = State((int) (vector_radius * cos(-pi * angle_limit / 180 + pi * iterator1 / num_of_vectors)),
                    (int) (vector_radius * sin(-pi * angle_limit / 180 + pi * iterator1 / num_of_vectors)),
                    (int) (curvature_max * fabs(cos(pi * min_cost_angle / 180))),
                    90);

            resultSeed->obstacleCostOfSeed = min_cost;
            resultSeed->velocityRatio = (1 + resultSeed->final_state.curvature()) / (1 - resultSeed->final_state.curvature());
            resultSeed->rightVelocity = vmax;
            resultSeed->leftVelocity = vmax * resultSeed->velocityRatio;
            State point((int) start.x(), (int) start.y(), 0, 0);

            resultSeed->intermediatePoints.insert(resultSeed->intermediatePoints.begin(), point);


            if (resultSeed == NULL) {
                status = 0;
                return std::make_pair(std::vector<State>(), Seed());
            }
            status = 999;
            return std::make_pair(resultSeed->intermediatePoints, *resultSeed);

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
