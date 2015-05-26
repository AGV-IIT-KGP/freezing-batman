#include <cmath>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <queue>

const int bot_x = 500, bot_y = 900;
int step_move = -700;
const cv::Point origin(0, 480); //Wrt top left corner
int count = 0;
int debug=1;
ros::Publisher pub_point;
geometry_msgs::Pose2D msge;
//queue<geometry_msgs::Pose2D> temp;
int counter=0;

geometry_msgs::Pose2D* temp=new geometry_msgs::Pose2D [4];


geometry_msgs::Pose2D findTarget(cv::Mat img) {
    cv::Mat cdst, mdst;
    mdst = img - img;
    std::vector<cv::Vec4i>lines;
    cv::Point center_point;
    center_point.x = 0, center_point.y = 0;
    double center_angle = 0.0;
    cdst = img;
    cv::Canny(img, cdst, 25, 75, 3);
    cv::HoughLinesP(cdst, lines, 1, CV_PI / 180, 25, 15, 5);
    int image_halfy=0;
    cv::Point top(0,0), bottom(0,0);
    int c=0;
    /*for(int i=0;i<img.rows;i++)
    {
        for(int j=0;j<img.cols;j++)
        {
            if(img.at<uchar>(i,j)>200)
            {
                image_halfy=i/2;
            }
        }
    }*/
    image_halfy=img.rows-image_halfy;
    for(int i=0/*image_halfy*/;i<img.rows;i++)
    {
        for(int j=0;j<img.cols;j++)
        {

            if(img.at<uchar>(i,j)>200 /*&& img.at<uchar>(i,j)[1]>200 && img.at<uchar>(i,j)[2]>200*/)
            {
                c++;
                top.x+=j;
                top.y+=i;
            }
            if(c==20)
                break;
        }
        if(c==20)
                break;
    }
    top.x/=20;
    top.y/=20;
    top.y-=img.rows;
    c=0;
    for(int i=img.rows-1;i>=0/*image_halfy*/;i--)
    {
        for(int j=0;j<img.cols;j++)
        {
            if(img.at<uchar>(i,j)>200 /*&& img.at<uchar>(i,j)[1]>200 && img.at<uchar>(i,j)[2]>200*/)
            {
                c++;
                bottom.x+=j;
                bottom.y+=i;
            }
            if(c==20)
                break;
        }
        if(c==20)
            break;
    }
    bottom.x/=20;
    bottom.y/=200;
    bottom.y-=img.rows;

    for (int i = 0; i < lines.size(); i++) {
        cv::Vec4i l = lines[i];
        cv::line(mdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255, 255, 255), 3, CV_AA);
    }

    for (int i = 0; i < lines.size(); i++) {
        cv::Vec4i p = lines[i];
        /*if(((p[1]+p[3])/2)>image_halfy)
            continue;*/
        center_point.x += (p[0] + p[2]) / 2;
        center_point.y += (p[1] + p[3]) / 2;
        if ((p[1] - p[3]) != 0) {
            double theta = std::atan2((p[1] - p[3]), (p[0] - p[2]));
            if (theta < 0)
                theta = 3.14 + theta;
            center_angle += (theta);
        }
        else
        {
            center_angle+=1.57;
        }
    }
    if (debug)std::cout << "Total Angle" << center_angle << std::endl;
    if (lines.size() != 0) {
        center_angle = center_angle / lines.size();
        center_point.x = center_point.x / lines.size();
        center_point.y = center_point.y / lines.size();
        if (debug)std::cout << center_point.x << " " << center_point.y << std::endl;
    }
    else
    {
	center_angle= 1.57;
	center_point.x= bot_x;
	center_point.y= bot_y;
    }  
    double m = tan(center_angle);
    if (m == HUGE_VAL) {
        m = 10000;
    }
    cv::Point leftCenter(0, 0), rightCenter(0, 0);
    double leftSlope = 0.0, rightSlope = 0.0, leftCount = 0, rightCount = 0;
    


    cv::Point proj,target;
    geometry_msgs::Pose2D target_pose;

    if((bottom.x-top.x)>90 || (top.x-bottom.x)>90)
    {
        
        if(bottom.x>top.x)
        {
            bottom.x-=40;
            top.x-=40;
            if(top.x<0)
                top.x=0;
            proj.x = (bot_x + m * (bot_y - center_point.y) + m * m * center_point.x) / (1 + m * m); // Verified
            proj.y = (center_point.y + m * (bot_x - center_point.x) + m * m * bot_y) / (1 + m * m); // Verify it
            target.x += proj.x + cos(center_angle) * step_move -40;
            if(target.x<0)
                target.x=0;
            target.y = m*(top.x-bottom.x)+bottom.y;
        }
        if(bottom.x<top.x)
        {
            bottom.x+=40;
            top.x+=40;
            if(top.x>img.cols)
                top.x=img.cols;
            proj.x = (bot_x + m * (bot_y - center_point.y) + m * m * center_point.x) / (1 + m * m); // Verified
            proj.y = (center_point.y + m * (bot_x - center_point.x) + m * m * bot_y) / (1 + m * m); // Verify it
            target.x += proj.x + cos(center_angle) * step_move +40;
            if(target.x>img.cols)
                target.x=img.cols;
            target.y = m*(top.x-bottom.x)+bottom.y;
        }
        
        center_angle = -1 * center_angle * 180 / CV_PI;
        //std::cout<<"new\n";
        target_pose.x = target.x;
        target_pose.y = (-1 * target.y + origin.y);
        target_pose.theta = center_angle;
    }


    else{
        for (int i = 0; i < lines.size(); i++) 
        {
            cv::Vec4i p = lines[i];
            cv::Point midPoint = cv::Point((p[0] + p[2]) / 2, (p[1] + p[3]) / 2);
            /*if(midPoint.y>image_halfy)
                continue;*/
            if(m==0)
                continue;
            double L11 = (0 - center_point.y) / m - (0 - center_point.x);
            double L22 = (midPoint.y - center_point.y) / m - (midPoint.x - center_point.x + 0.0);

            if (L11 * L22 > 0) {
                leftCenter.x += midPoint.x;
                leftCenter.y += midPoint.y;
                if ((lines[i][0] - lines[i][2]) != 0) {
                    leftSlope += -(lines[i][1] - lines[i][3]) / (lines[i][0] - lines[i][2]);
                }
                leftCount++;
            } 
            else {
                rightCenter.x += midPoint.x;
                rightCenter.y += midPoint.y;
                if ((lines[i][0] - lines[i][2]) != 0) {
                    rightSlope += -(lines[i][1] - lines[i][3]) / (lines[i][0] - lines[i][2]);
                }
                rightCount++;
            }
        }
        if(leftCount!=0 && rightCount!=0){
            leftCenter.x /= leftCount; //Load when count==0
            leftCenter.y /= leftCount;
            leftSlope /= leftCount;

            rightCenter.x /= rightCount;
            rightCenter.y /= rightCount;
            
            rightSlope /= rightCount;
            if ((center_point.x - leftCenter.x) < 50 || (leftCenter.x - center_point.x) < 50) {
                center_point.x += 150; //Target must not lie on the lane
                target.x = 150;
            }
            if ((rightCenter.x - center_point.x) < 50 || (center_point.x - rightCenter.x) < 50) {
                center_point.x = center_point.x - 150;
                target.x -= 150;
            }
        }

        proj.x = (bot_x + m * (bot_y - center_point.y) + m * m * center_point.x) / (1 + m * m); // Verified
        proj.y = (center_point.y + m * (bot_x - center_point.x) + m * m * bot_y) / (1 + m * m); // Verify it
        target.x += proj.x + cos(center_angle) * step_move;
        target.y = proj.y + sin(center_angle) * step_move;
        cv::Point point2;
        point2.x = center_point.x - 100;
        point2.y = center_point.y - m * 100;
        
        center_angle = -1 * center_angle * 180 / CV_PI;
        
        target_pose.x = target.x;
        target_pose.y = (-1 * target.y + origin.y);
        target_pose.theta = center_angle;
        //cv::Point a(target_pose.x,target_pose.y);
        //cv::circle(img,a,2,cv::Scalar(0,0,255),1,8,0);
        //cv::namedWindow("new",CV_WINDOW_AUTOSIZE);
        //imshow("new!",img);
    }


    /*if(old_target.x!=0 || old_target.y!=0 || old_target.theta!=0)
    {
        target_pose.x=0.8*float(target_pose.x) + 0.2*float(old_target.x);
        target_pose.theta=0.8*float(target_pose.theta) + 0.2*float(old_target.theta);
        std::cout<<"in";
        target.x=target_pose.x;
        target.y=origin.y - target_pose.y;
        if(target.y<60 || target.y>700)
            target.y=230;
    }*/
    double valx=target_pose.x;
    double valy=target_pose.y;
    int valtheta=target_pose.theta;
    for(int i=0;i<counter;i++)
    {
        valx+=(i+2)*(temp[i]).x;
        valy+=(i+2)*(temp[i]).y;
        valtheta+=(i+2)*(temp[i]).theta;
    }
    target_pose.x=int(valx/15);
    target_pose.y=int(valy/15);
    target_pose.theta=int(valtheta/15);

    target.x=target_pose.x;
    target.y=origin.y - target_pose.y;
    if(target.y<60 || target.y>700)
    {   target.y=230;
        target_pose.y=origin.y - target.y;
    }

    if (debug)
    {
        std::cout << "target.x: " << target.x << " target.y: " << target.y << std::endl;
        std::cout << "proj.x: " << proj.x << " " << "proj.y: " << proj.y << std::endl;
        //cv::line(mdst, proj, target, cv::Scalar(150),2,8);
        cv::line(mdst, cv::Point(bot_x, bot_y), target, cv::Scalar(255), 2, 8);
        cv::namedWindow("Center_path", cv::WINDOW_NORMAL);
        cv::imshow("Center_path", mdst);
        // cv::waitKey(0);
    }
    return target_pose;

}

void publishTarget(const sensor_msgs::ImageConstPtr msg ) {
    if (debug){ROS_INFO("Listened for the %d time\n", count++);
    cv::namedWindow("listening", CV_WINDOW_AUTOSIZE);}
    cv::Mat img;
    

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    // img= cv_bridge.imgMsgToCv(msge, "mono8");
    // cv::cvtColor(msg.image,img,CV_BGR2GRAY);
    img = cv_ptr->image;
    msge = findTarget(img);
    geometry_msgs::Pose2D temp1,temp2;
    temp1=msge;
    for(int i=0;i<4;i++)
    {
        temp2=temp[i];
        temp[i]=temp1;
        temp1=temp2;
    }
    pub_point.publish(msge);
    if(debug)
    {
       cv::waitKey(33);
       ROS_INFO("%f %f %f ", msge.x, msge.y, msge.theta);
    }
    if(counter<4)
        counter++;
}

int main(int argc, char **argv) {
    std::string node_name= "lane_navigator";
    ros::init(argc, argv, node_name);
    ros::NodeHandle node_handle;
    msge.x=0;
    msge.y=0;
    msge.theta=0;
    pub_point = node_handle.advertise<geometry_msgs::Pose2D>("/lane_navigator/proposed_target", 50);
    ros::Subscriber lanes_subscriber = node_handle.subscribe("/lane_detector1/lanes", 1, &publishTarget);
    while(ros::ok()){
    node_handle.getParam(node_name + "/debug", debug);
    ros::spinOnce();
    }
    return 0;
}
