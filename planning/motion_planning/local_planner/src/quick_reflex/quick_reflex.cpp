#include "quick_reflex/quick_reflex.hpp"

const int INFINITE = 99999;

namespace navigation {

    std::pair<std::vector<State>, Seed> quickReflex::findPathToTarget(const cv::Mat& img, const State& start, const State& goal) {

        givenSeeds.clear();
        loadGivenSeeds(start, goal);

        fusionMap = img;

        if (start.isCloseTo(goal)) {
            printf("Bot is On Target\n");
            return std::make_pair(std::vector<State>(), Seed());
        }
        
        else if(isOnTheObstacle(goal)){
            printf("Bot is on the Obstacle Map\n");
            return std::make_pair(std::vector<State>(), Seed());
        }
        else if(isOnTheObstacle(goal)){
            printf("Target is on the Obstacle Map\n");
            return std::make_pair(std::vector<State>(), Seed());
        }

        Seed* resultSeed = NULL;
        double minCost = INFINITE;

        std::vector<Seed> neighbours = neighborNodesWithSeeds(start, goal);

        for(int i = 0; i < neighbours.size(); ++i){

            if(neighbours[i].costOfseed < minCost){

                minCost = neighbours[i].costOfseed;
                resultSeed = &neighbours[i];
            }
        }

        if(resultSeed == NULL){

            printf("NO PATH FOUND\n");
            return std::make_pair(std::vector<State>(), Seed());
        }

        return std::make_pair(resultSeed->intermediatePoints, *resultSeed);
    }
}
