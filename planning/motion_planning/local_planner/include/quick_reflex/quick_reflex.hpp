
#include <stdio.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <sstream>
#include <queue>
#include <map>
#include <string>

#include "seed.hpp"
#include "state.hpp"

namespace navigation {

    struct comparatorMapState_   {


        inline bool operator()(const State& a, const State& b)   {
                double k11 = a.x();
                double k12 = a.y();
                double k13 = a.theta();
                
                double cantor11 = 0.5 * (k11 + k12) * (k11 + k12 + 1) + k12;
                double cantor12 = 0.5 * (cantor11 + k13) * (cantor11 + k13 + 1) + k13;
                
                double k21 = b.x();
                double k22 = b.y();
                double k23 = b.theta();
                
                double cantor21 = 0.5 * (k21 + k22) * (k21 + k22 + 1) + k22;
                double cantor22 = 0.5 * (cantor21 + k23) * (cantor21 + k23 + 1) + k23;
                
                return cantor12 < cantor22;

        }
    };

    
    class quickReflex
    {

    public:
        quickReflex();
        std::pair<std::vector<State>, Seed> findPathToTarget(const cv::Mat& fusionMap, const State& start, const State& goal);
        void showPath(std::vector<State>& path, const State& start, const State& goal);

    private:
        cv::Mat fusionMap;
        cv::Mat image;
        const std::string SEEDS_FILE;
        std::vector<Seed> givenSeeds;
        void distanceTransform();
        bool isOnTheObstacle(const State& state);
        void loadGivenSeeds(const State& start, const State& goal);
        void plotPointInMap(const State& pos_) ;
        std::vector<Seed> neighborNodesWithSeeds(const State&  start,const State&  goal)  ;
        bool onTarget(State const& currentState_, const State& targetState)  ;
        bool isWalkableWithSeeds(State const& startState_, State const& nextState_, Seed targetSeed) ;

    };
}