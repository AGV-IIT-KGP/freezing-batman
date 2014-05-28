#include <a_star_seed/a_star_seed.hpp>

const int INFINITE = 99999;

namespace navigation {

    std::pair<std::vector<State>, Seed> quickReflex::findPathToTarget(const cv::Mat& img, const State& start, const State& goal, int& status) {
        fusion_map = img;
        givenSeeds.clear();
        distanceTransform();
//        cv::namedWindow("view", CV_WINDOW_FREERATIO);
//        cv::imshow("view", fusion_map);
//        cv::waitKey(0);
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
        double minObstacleCost = INFINITE;
        std::vector<Seed> neighbours = neighborNodesWithSeeds(start, goal);
        int j = 0;
        for (int i = 0; i < neighbours.size(); ++i) {
//            std::cout << neighbours[i].obstacleCostOfSeed << " " << i << " " << minObstacleCost << std::endl;
            if (neighbours[i].obstacleCostOfSeed < minObstacleCost) {
                minObstacleCost = neighbours[i].obstacleCostOfSeed;
                j = i;
                resultSeed = &neighbours[i];
            }
        }
        //ROS_INFO("resultSeed: %d, %lf, %lf", j, neighbours[j].leftVelocity, neighbours[j].rightVelocity);

        double minTargetCost = INFINITE;
//        std::cout << neighbours.size() << " " << givenSeeds.size() << std::endl;
        if (neighbours.size() >= givenSeeds.size() *.98) {
            for (int i = 0; i < neighbours.size(); ++i) {
                if (neighbours[i].targetCostOfSeed < minTargetCost) {
                    minTargetCost = neighbours[i].targetCostOfSeed;
                    resultSeed = &neighbours[i];
                }
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
