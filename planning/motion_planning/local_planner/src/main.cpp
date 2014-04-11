#include <sys/time.h>
#include <sstream>
#include <iostream>

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

#define LEFT_CMD 0
#define RIGHT_CMD 1

#define MAP_MAX 1000
#define LOOP_RATE 5000
#define WAIT_TIME 100

int ol_overflow;
//geometry_msgs::Twist precmdvel;
int last_cmd;

/* make 2d array char**
subscribe image convert to mat
then update char ** local map : DONE*/

char local_map[1000][1000];
navigation::State my_bot_location, my_target_location;
navigation::State pose;


void update_world_map(const sensor_msgs::ImageConstPtr& world_map){
    //TODO : copy function for occupancy grid
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(world_map, sensor_msgs::image_encodings::BGR8);
      cv::Mat bin = cv::Mat::zeros(cv_ptr->image.size(),CV_8UC1);
      cv::cvtColor(cv_ptr->image,bin,CV_BGR2GRAY);
      for(int i=0;i<cv_ptr->image.rows;i++){
        for(int j=0;j<cv_ptr->image.cols;j++){
            local_map[i][j] = (char)('0'+cv_ptr->image.at<uchar>(i,j));
        }
      }

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


    ros::NodeHandle nh; // nodeHandle
    ros::Publisher pub_path = nh.advertise<local_planner::Seed>("/path", 1000); //Publisher for Path

    ros::Subscriber sub_world_map = nh.subscribe("/world_map",10, update_world_map); //Subscriber for World Map
    ros::Subscriber sub_bot_pose =  nh.subscribe("/bot_pose", 10 ,update_bot_pose); // topic should same with data published by GPS
    ros::Subscriber sub_target_pose = nh.subscribe("/target_Pose", 10 , update_target_pose); // topic published from GPS
    // ros::Subscriber sub3 = nh.subscribe("/pose", 1, update_pose);

    /* TO DO
    Custom Message for array of pose3D (check ROS tut).*/
   // pub_path = nh.advertise<geometry_msgs::Twist > ("cmd_vel", 1);

    cvNamedWindow("[PLANNER] Map", 0);

    navigation::State botLocation(500,100,90,0),targetLocation(900,900,90,0);
    navigation::AStarSeed planner;

    ros::Rate loop_rate(LOOP_RATE);

    srand((unsigned int)time(NULL));

          struct timeval t,c;

    
    int iterations = 100;
        gettimeofday(&t,NULL);

    while (iterations-- &&  ros::ok()) {
        

              local_planner::Seed seed;

        std::pair<std::vector<navigation::StateOfCar>, navigation::Seed> path;

        cv::Mat img = cv::Mat::zeros(1000, 1000, CV_8UC1);


        // std::chrono::steady_clock::time_point startC=std::chrono::steady_clock::now();
        // navigation::addObstacles(img, 5);


             path = planner.findPathToTargetWithAstar(img,botLocation, targetLocation);
             // planner.showPath(path.first);

        




        planner.showPath(path.first);
        for(int i = 0; i < path.first.size(); i++){
            ROS_INFO("%d %d \n", path.first[i].x(), path.first[i].y());
        }
        

        seed.x = path.second.finalState.x();
        seed.y = path.second.finalState.y();
        seed.theta = path.second.finalState.theta();
        seed.costOfseed = path.second.costOfseed;
        seed.velocityRatio = path.second.velocityRatio;
        seed.leftVelocity = path.second.leftVelocity;
        seed.rightVelocity = path.second.rightVelocity;
        seed.curvature = path.second.finalState.curvature();
        
        pub_path.publish(seed);

        ros::spinOnce();
        loop_rate.sleep();

    }

        gettimeofday(&c,NULL);
        double td = t.tv_sec + t.tv_usec/1000000.0;
        double cd = c.tv_sec + c.tv_usec/1000000.0; // time in seconds for thousand iterations

    
        std::cout<<"FPS:"<< 100/(cd-td) <<std::endl;


    ROS_INFO("Planner Exited");

    return 0;
}
