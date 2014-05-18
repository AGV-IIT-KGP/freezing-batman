#include "a_star_seed/a_star_seed.hpp"

const int INFINITE = 99999;
const int LOOP_RATE = 10;

namespace navigation {
     void quickReflex::publishStatus(ros::NodeHandle& nh , int status){
        ros:: Publisher status_pub = nh.advertise<std_msgs::String>("localplanner/status",1000);
        ros::Rate loop_rate(LOOP_RATE);
        std_msgs::String msg;

        std::stringstream ss;
        if(status == 0){
            ss << "NO PATH FOUND";
        }
        else if(status == 1)
        {
            ss << "BOT ON TARGET";
        }
        else {
            ss << "A PATH IS FOUND";
        }

        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        status_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    std::pair<std::vector<State>, Seed> quickReflex::findPathToTarget(const cv::Mat& img, const State& start, const State& goal) {

        fusionMap = img;
        distanceTransform();

        givenSeeds.clear();
        loadGivenSeeds(start, goal);

        if (start.isCloseTo(goal)) {
            publishStatus(nh, 1);
            return std::make_pair(std::vector<State>(), Seed());
        }
        
        else if(isOnTheObstacle(start)){
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

        publishStatus(nh, 0);

        return std::make_pair(std::vector<State>(), Seed());

        }

        return std::make_pair(resultSeed->intermediatePoints, *resultSeed);
    }

   
}
