#include <a_star_seed/a_star_seed.hpp>

const int INFINITE = 99999;

namespace navigation {

    std::pair<std::vector<State>, Seed> quickReflex::findPathToTarget(const cv::Mat& img, const State& start, const State& goal, int& status) {
        fusion_map = img;
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
        double minCost = INFINITE;
        std::vector<Seed> neighbours = neighborNodesWithSeeds(start, goal);
        for (int i = 0; i < neighbours.size(); ++i) {
            if (neighbours[i].costOfseed < minCost) {
                minCost = neighbours[i].costOfseed;
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
