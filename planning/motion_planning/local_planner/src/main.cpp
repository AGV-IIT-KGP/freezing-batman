#include <sys/time.h>
#include <sstream>
#include <iostream>
#include <time.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cvaux.hpp>
#include <opencv/cxcore.h>
#include <cv.h>
#include <highgui.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"

#include "local_planner/Seed.h"
#include "a_star_seed/a_star_seed.hpp"

#define LOOP_RATE 10
#define WAIT_TIME 100



cv::Mat local_map;
navigation::State my_bot_location, my_target_location;
navigation::State pose;
ros::Publisher pub_path;
navigation::State botLocation(500,100,90,0);
navigation::State targetLocation(500,900,90,0);
navigation::AStarSeed planner;

void callback(){

    local_planner::Seed seed;
    std::pair<std::vector<navigation::StateOfCar>, navigation::Seed> path;

        cv::Mat img = cv::Mat::zeros(cv::Size(1000,1000), CV_8UC1);
         // std::chrono::steady_clock::time_point startC=std::chrono::steady_clock::now();
            // navigation::addObstacles(img, 5);
          
        struct timeval t,c;
        gettimeofday(&t,NULL);
          
        path = planner.findPathToTargetWithAstar(img , botLocation, targetLocation);
        std::cout<<__LINE__<<std::endl;
        planner.showPath(path.first);
           // if(path.finalState.x()==0)
          // if(path.velocityRatio == 0)
          //   continue;
        seed.x = path.second.finalState.x();
        seed.y = path.second.finalState.y();
        seed.theta = path.second.finalState.theta();
        seed.costOfseed = path.second.costOfseed;
        seed.velocityRatio = path.second.velocityRatio;
        seed.leftVelocity = path.second.leftVelocity;
        seed.rightVelocity = path.second.rightVelocity;
        seed.curvature = path.second.finalState.curvature();
         
        pub_path.publish(seed);
        gettimeofday(&c,NULL);
        double td = t.tv_sec + t.tv_usec/1000000.0;
        double cd = c.tv_sec + c.tv_usec/1000000.0; // time in seconds for thousand iterations
        std::cout<<"FPS:"<< 1/(cd-td) <<std::endl;

}
void update_world_map(const sensor_msgs::ImageConstPtr world_map){
    //TODO : copy function for occupancy grid
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(world_map, sensor_msgs::image_encodings::MONO8);
        local_map = cv_ptr->image.clone();
        
        // cv::cvtColor(cv_ptr->image,local_map,CV_BGR2GRAY);
                
        local_planner::Seed seed;
        std::pair<std::vector<navigation::StateOfCar>, navigation::Seed> path;

        cv::Mat img = local_map.clone();
         // std::chrono::steady_clock::time_point startC=std::chrono::steady_clock::now();
            // navigation::addObstacles(img, 5);
          
        std::cout<<"Got Map at line : "<<__LINE__<<std::endl;
        struct timeval t,c;
        gettimeofday(&t,NULL);
          
        std::cout<<__LINE__<<std::endl;
        path = planner.findPathToTargetWithAstar(img , botLocation, my_target_location);
        std::cout<<__LINE__<<std::endl;
        planner.showPath(path.first);
           // if(path.finalState.x()==0)
          // if(path.velocityRatio == 0)
          //   continue;
        seed.x = path.second.finalState.x();
        seed.y = path.second.finalState.y();
        seed.theta = path.second.finalState.theta();
        seed.costOfseed = path.second.costOfseed;
        seed.velocityRatio = path.second.velocityRatio;
        seed.leftVelocity = path.second.leftVelocity;
        seed.rightVelocity = path.second.rightVelocity;
        seed.curvature = path.second.finalState.curvature();
         
        std::cout<<__LINE__<<std::endl;
        pub_path.publish(seed);
        std::cout<<__LINE__<<std::endl;
        gettimeofday(&c,NULL);
        double td = t.tv_sec + t.tv_usec/1000000.0;
        double cd = c.tv_sec + c.tv_usec/1000000.0; // time in seconds for thousand iterations
        std::cout<<"FPS:"<< 1/(cd-td) <<std::endl;

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
}

void update_bot_pose(const geometry_msgs::Pose::ConstPtr _pose){
    int x = _pose->position.x;
    int y = _pose->position.y;
    int z = _pose->position.z;
    my_bot_location = navigation::State(x,y,z,0);
}
void update_target_pose(const geometry_msgs::Pose::ConstPtr _pose){
    int x = _pose->position.x;
    int y = _pose->position.y;
    int z = _pose->position.z;
    ROS_INFO("%d %d %d \n", x, y, z);
    callback();
    my_target_location = navigation::State(x,y,z,0);
}
void update_pose(const geometry_msgs::Pose::ConstPtr _pose){
    int x = _pose->position.x;
    int y = _pose->position.y;
    int z = _pose->position.z;
    pose = navigation::State(x,y,z,0);
}


int main(int argc,char* argv[]) {

    ros::init(argc, argv, "planner");

    std::cout<<"Entered Planner \n";

    ros::NodeHandle nh; // nodeHandle

    pub_path= nh.advertise<local_planner::Seed>("/path", 1000); //Publisher for Path

    // ros::Subscriber sub_world_map = nh.subscribe("/world_map",1000, update_world_map); //Subscriber for World Map
    ros::Subscriber sub_bot_pose =  nh.subscribe("/bot_pose", 1000 ,update_bot_pose); // topic should same with data published by GPS
    ros::Subscriber sub_target_pose = nh.subscribe("/target_pose", 1000 , update_target_pose); // topic published from GPS
 
    // ros::Subscriber sub3 = nh.subscribe("/pose", 1, update_pose);

    /* TO DO
    Custom Message for array of pose3D (check ROS tut).*/
   // pub_path = nh.advertise<geometry_msgs::Twist > ("cmd_vel", 1);

    cvNamedWindow("[PLANNER] Map", 0);
    std::cout<<__LINE__<<std::endl;

    ros::Rate loop_rate(LOOP_RATE);

    srand((unsigned int)time(NULL));
    int iterations = 100;

    while (iterations--) {
        // ROS_INFO("Iter number : %d\n",iterations);
        callback();
        // ros::spinOnce();
        loop_rate.sleep();

    }
    ROS_INFO("Planner Exited");

    return 0;

}
