#include "backup_planner.hpp"
#include <sys/time.h>

namespace navigation {

    backupPlanner::backupPlanner(ros::NodeHandle& nodeHandle) : nh(nodeHandle) {
        //Subscriber for World Map
        sub_world_map = nh.subscribe("interpreter/fusion/world_map", 10, &backupPlanner::updateWorldMap, this);
        // topic should same with data published by GPS
        sub_bot_pose = nh.subscribe("/bot_pose", 10, &backupPlanner::updateBotPose, this);
        // topic published from GPS
        sub_target_pose = nh.subscribe("/target_pose", 10, &backupPlanner::updateTargetPose, this);

        pub_path = nh.advertise<backup_planner::Seed>("/path", 1000); //Publisher for Path

        local_map = cv::Mat::zeros(MAP_MAX, MAP_MAX, CV_8UC1);

        // navigation::addObstacles(local_map, 5);
        ROS_INFO("Local Planner(AStarSeed) started.... ");
        ROS_INFO("Publisher : \"/path\" .... ");
        ROS_INFO("Subscriber : \"interpreter/fusion/world_map\", \"/bot_pose\", \"/target_pose\" .... ");
    }

    void backupPlanner::updateWorldMap(const sensor_msgs::ImageConstPtr& world_map) {
        //TODO : copy function for occupancy grid
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(world_map, sensor_msgs::image_encodings::MONO8);
            local_map = cv_ptr->image;
        }        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

    inline void backupPlanner::updateBotPose(const geometry_msgs::Pose::ConstPtr _pose){
        int x = _pose->position.x;
        int y = _pose->position.y;
        int z = _pose->position.z;
        my_bot_location = navigation::State(x,y,z,0);
    }

    inline void backupPlanner::updateTargetPose(const geometry_msgs::Pose::ConstPtr _pose){
        int x = _pose->position.x;
        int y = _pose->position.y;
        int z = _pose->position.z;
        my_target_location = navigation::State(x,y,z,0);
    }

    void backupPlanner::plan() {
        ros::Rate loop_rate(LOOP_RATE);
        navigation::AStarSeed planner(std::string(""));

        while (ros::ok()) {
            ros::spinOnce();

            struct timeval t,c;
            gettimeofday(&t,NULL);
            //std::pair<std::vector<navigation::StateOfCar>, navigation::Seed> path =
            //        planner.findPathToTargetWithAstar(local_map, my_bot_location, my_target_location);
            gettimeofday(&c,NULL);
            double td = t.tv_sec + t.tv_usec/1000000.0;
            double cd = c.tv_sec + c.tv_usec/1000000.0;
            std::cout<<"FPS:"<< 1/(cd-td) <<std::endl;
            planner.showPath(path.first, my_bot_location, my_target_location);

            publishData(path);
            loop_rate.sleep();
        }
    }

    void backupPlanner::publishData(std::pair<std::vector<navigation::StateOfCar>, navigation::Seed>& path) {
        local_planner::Seed seed;

        seed.x = path.second.finalState.x();
        seed.y = path.second.finalState.y();
        seed.theta = path.second.finalState.theta();
        seed.costOfseed = path.second.costOfseed;
        seed.velocityRatio = path.second.velocityRatio;
        seed.leftVelocity = path.second.leftVelocity;
        seed.rightVelocity = path.second.rightVelocity;
        seed.curvature = path.second.finalState.curvature();

        pub_path.publish(seed);
    }
}