#include "ros/ros.h"
#include <sstream>
#include <std_msgs/String.h>
#include <cmath>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/image_encodings.h>

const int bot_x = 320, bot_y = 450;
int step_move = -400;
const cv::Point origin(0, 480); //Wrt top left corner
int count = 0;
int debug=0;  //Temporary

geometry_msgs::Pose2D target_finder(cv::Mat img) {
    geometry_msgs::Pose2D p;
    cv::Mat cdst, mdst;
    mdst = img - img;
    std::vector<cv::Vec4i>lines;
    cv::Point center_point;
    center_point.x = 0, center_point.y = 0;
    double center_angle = 0.0;
    cdst = img;
    cv::Canny(img, cdst, 25, 75, 3);
    cv::HoughLinesP(cdst, lines, 1, CV_PI / 180, 25, 15, 5);
    for (int i = 0; i < lines.size(); i++) {
        cv::Vec4i l = lines[i];
        line(mdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255, 255, 255), 3, CV_AA);
    }

    for (int i = 0; i < lines.size(); i++) {
        cv::Vec4i p = lines[i];
        center_point.x += (p[0] + p[2]) / 2;
        center_point.y += (p[1] + p[3]) / 2;
        if ((p[1] - p[3]) != 0) {
            double theta = std::atan2((p[1] - p[3]), (p[0] - p[2]));
            if (theta < 0)
                theta = 3.14 + theta;
            center_angle += (theta);
        }
    }

    if(debug)std::cout << "Total Angle" << center_angle << std::endl;
    if (lines.size() != 0) {
        center_angle = center_angle / lines.size();
        center_point.x = center_point.x / lines.size();
        center_point.y = center_point.y / lines.size();
        if(debug)std::cout << center_point.x << " " << center_point.y << std::endl;
    }
    double m = tan(center_angle);
    if (m == HUGE_VAL) {
        m = 10000;
    }
  
    cv::Point leftCenter(0, 0), rightCenter(0, 0);

    double leftSlope = 0.0, rightSlope = 0.0, leftCount = 0, rightCount = 0;
    for (int i = 0; i < lines.size(); i++) {
        cv::Vec4i p = lines[i];
        cv::Point midPoint = cv::Point((p[0] + p[2]) / 2, (p[1] + p[3]) / 2);
        double L11 = (0 - center_point.y) / m - (0 - center_point.x);
        double L22 = (midPoint.y - center_point.y) / m - (midPoint.x - center_point.x + 0.0);

        if (L11 * L22 > 0) {
            leftCenter.x += midPoint.x;
            leftCenter.y += midPoint.y;
            if ((lines[i][0] - lines[i][2]) != 0) {
                leftSlope += -(lines[i][1] - lines[i][3]) / (lines[i][0] - lines[i][2]);
            }
            leftCount++;
        } else {
            rightCenter.x += midPoint.x;
            rightCenter.y += midPoint.y;
            if ((lines[i][0] - lines[i][2]) != 0) {
                rightSlope += -(lines[i][1] - lines[i][3]) / (lines[i][0] - lines[i][2]);
            }
            rightCount++;
        }
    }

    leftCenter.x /= leftCount; //Load when count==0
    leftCenter.y /= leftCount;
    leftSlope /= leftCount;

    rightCenter.x /= rightCount;
    rightCenter.y /= rightCount;
    cv::Point target, proj;
    rightSlope /= rightCount;

    if ((center_point.x - leftCenter.x) < 50 || (leftCenter.x - center_point.x) < 50) {
        center_point.x += 150; //Target must not lie on the lane
        target.x = 150;
    } 

    if ((rightCenter.x - center_point.x) < 50 || (center_point.x - rightCenter.x) < 50) {
        center_point.x = center_point.x - 150;
        target.x -= 150;
    }



    proj.x = (bot_x + m * (bot_y - center_point.y) + m * m * center_point.x) / (1 + m * m); // Verified
    proj.y = (center_point.y + m * (bot_x - center_point.x) + m * m * bot_y) / (1 + m * m); // Verify it
    target.x += proj.x + cos(center_angle) * step_move;
    target.y = proj.y + sin(center_angle) * step_move;

    cv::Point point2;
    point2.x = center_point.x - 100;
    point2.y = center_point.y - m * 100;

   if(debug)std::cout << "target.x: " << target.x << " target.y: " << target.y << std::endl;
   if(debug)std::cout << "proj.x: " << proj.x << " " << "proj.y: " << proj.y << std::endl;

    //cv::line(img, proj, target, cv::Scalar(150),2,8);
    cv::line(mdst, cv::Point(bot_x, bot_y), target, cv::Scalar(255), 2, 8);
    cv::namedWindow("Center_path", cv::WINDOW_AUTOSIZE);
    cv::imshow("Center_path", mdst);
    // cv::waitKey(0);
    center_angle = -1 * center_angle * 180 / CV_PI;
    p.x = target.x;
    p.y = -1 * target.y + origin.y;
    p.theta = center_angle;
    return p;
}

void imageCallback(const sensor_msgs::ImageConstPtr msg) {
    cv::namedWindow("listening", CV_WINDOW_AUTOSIZE);
    //cv::imshow("listening", msg.image);
    cv::waitKey(10);
    if(debug)ROS_INFO("Listened for the %d time\n", count++);
    ros::NodeHandle n;
    ros::Publisher pub_point = n.advertise<geometry_msgs::Pose2D>("target_point", 50);
    ros::Rate loop_rate(10);
    while (n.ok()) {
        cv::Mat img;
        geometry_msgs::Pose2D msge;
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        // img= cv_bridge.imgMsgToCv(msge, "mono8");
        // cv::cvtColor(msg.image,img,CV_BGR2GRAY);
        img = cv_ptr->image;
        msge = target_finder(img);
        cv::waitKey(33);
        pub_point.publish(msge);
        if(debug)ROS_INFO("%f %f %f ", msge.x, msge.y, msge.theta);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "targetFromLane");
    ros::NodeHandle n;
 
    ros::Subscriber sub = n.subscribe("/interpreter/lane_detector/0", 5, &imageCallback);
    ros::spin();

    return 0;
}
