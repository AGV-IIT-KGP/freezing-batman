#include "planner.h"
#include <sstream>
#include <nav_msgs/OccupancyGrid.h> 
int ol_overflow;
//geometry_msgs::Twist precmdvel;
int last_cmd;

/* make 2d array char**
subscribe image convert to mat
then update char ** local map : DONE*/

char local_map[1000][1000];
Triplet my_bot_location, my_target_location;
Pose pose;


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
    my_bot_location.x = _pose->position.x;
    my_bot_location.y = _pose->position.y;
    my_bot_location.z = _pose->position.z;
}
void update_target_pose(const geometry_msgs::Pose::ConstPtr _pose){
    my_target_location.x = _pose->position.x;
    my_target_location.y = _pose->position.y;
    my_target_location.z = _pose->position.z;
}
void update_pose(const geometry_msgs::Pose::ConstPtr _pose){
    pose.position.x = _pose->position.x;
    pose.position.x = _pose->position.y;
    pose.position.x = _pose->position.z;
}


int main(int argc,char* argv[]) {

    ros::init(argc, argv, "planner");

    ROS_INFO("At line %d\n",__LINE__);
    ros::NodeHandle nh; // nodeHandle
    ros::Publisher pub_path; //Publisher for Path

    ROS_INFO("At line %d\n",__LINE__);

    ros::Subscriber sub_world_map = nh.subscribe("/world_map",10, update_world_map); //Subscriber for World Map
    ros::Subscriber sub_bot_pose =  nh.subscribe("/bot_pose", 10 ,update_bot_pose); // topic should same with data published by GPS
    ros::Subscriber sub_target_pose = nh.subscribe("/target_Pose", 10 , update_target_pose); // topic published from GPS
    ros::Subscriber sub3 = nh.subscribe("/pose", 1, update_pose);

    ROS_INFO("At line %d\n",__LINE__);
    /* TO DO
    Custom Message for array of pose3D (check ROS tut).*/
   // pub_path = nh.advertise<geometry_msgs::Twist > ("cmd_vel", 1);

    cvNamedWindow("[PLANNER] Map", 0);

   planner_space::Planner::loadPlanner(argv);

       ROS_INFO("At line %d\n",__LINE__);

    ros::Rate loop_rate(LOOP_RATE);

    while (ros::ok()) {
        cv::Mat map_img(MAP_MAX, MAP_MAX, CV_8UC1, cv::Scalar(0));


      std::vector<Triplet> path = planner_space::Planner::findPath(my_bot_location, my_target_location, map_img);
        //pub_path.p 

        loop_rate.sleep();
    }

    ROS_INFO("Planner Exited");

    return NULL;
}
