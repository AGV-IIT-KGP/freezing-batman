#define _USE_MATH_DEFINES
#include <cmath>
#include <ros/ros.h>
#include <string>
#include <opencv/cv.h>
#include <opencv/highgui.h>

const int bot_x=320, bot_y=480;
const int step_move= 400;

// You were using 2 space indentation, I have fixed it. Fixing indentation in
// vim is just 2 keystroke difficult :P. The comman is `=G`
void lanesep(cv::Mat img)
{
    cv::Mat cdst;
    cv::namedWindow("Original", cv::WINDOW_AUTOSIZE);
    std::vector<cv::Vec4i>lines;
    cv::Point center_point;
    center_point.x=0, center_point.y=0;
    double center_angle=0.0;
    cv::HoughLinesP(img, lines, 1, CV_PI/180, 10, 5, 60);
    // How were the parameters choosen?
    // Do we really need this high resolution pi/1000?

    for( int i = 0; i < lines.size(); i++ )
    {
        cv::Vec4i l = lines[i];
        line(img, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255,255,255), 3, CV_AA);
    }

    cv::imshow("Original",img);
    cv::waitKey(0);

    for(int i=0;i<lines.size(); i++)
    {
        cv::Vec4i p=lines[i];
        center_point.x += (p[0]+p[2])/2;
        center_point.y += (p[1]+p[3])/2;
        if((p[1]-p[3])!=0){
            double theta = std::atan2((p[1] - p[3]), (p[0] - p[2]));
            // Variable name for angle was 'r' WTF!!
            center_angle -= fabs(theta);
            if(fabs(theta)<1) std::cout<<"Angle: "<<fabs(theta)<<std::endl;
            // What's so special about 1 radian?? Why not print all angles
        }
    }

    std::cout<<"Total Angle"<<center_angle<<std::endl;
    center_angle= center_angle/lines.size();
    std::cout<<"Average Angle"<<center_angle<<std::endl;

    center_point.x= center_point.x/lines.size();
    center_point.y= center_point.y/lines.size();
    std::cout<<center_point.x<<" "<<center_point.y<<std::endl;

    // Earlier: double m = (center_angle != 0) ? tan(center_angle) : 100000;
    // You probably did this to avoid overflow but tan(0) doesn't tends to
    // infinity tan(pi/2) does. See: `man tan`
    // And as a reminder you don't generally test for equality in floating point numbers
    // they are not exact, you check if the variable lies in value +- epsilon
    double m = tan(center_angle);
    if(m == HUGE_VAL){
        m = 10000; // I'm if we need to do this, check the value of HUGE_VAL
    }
    std::cout<<"slope: "<<m<<std::endl;
    std::cout<<"slope: "<<m<<std::endl;
    cv::Point leftCenter(0,0), rightCenter(0, 0);

    double leftSlope = 0.0, rightSlope = 0.0, leftCount = 0, rightCount = 0;
    for(int i = 1; i < lines.size(); i++) // Why does the counter starts from 1 here??
    {
        cv::Vec4i p = lines[i];
        cv::Point midPoint = cv::Point((p[0] + p[2])/2, (p[1] + p[3])/2);
        // Choose a better name bro, L11 !!
        //Earlier: double L11 = (0 - center_point.y + m*(0 - center_point.x + 0.0))/m;
        double L11 = (0 - center_point.y)/m - (0 - center_point.x); // the equation is (y - y1) - m*(x - x1) = 0
        // Why add a 0.0 in m*(0 - center_point.x + 0.0) ?
        // Why divide the whole by m? To avoid overflow? If yes you have not done it
        // right.
        //Earlier: double L22 = (midPoint.y - center_point.y + m*(midPoint.x - center_point.x + 0.0))/m;
        double L22 = (midPoint.y - center_point.y)/m - (midPoint.x - center_point.x + 0.0);
        // OK so you are testing if the 0, 0 and mid point lies on the same
        // side of the line.
        if(L11*L22 > 0)
        {
            leftCenter.x += midPoint.x;
            leftCenter.y += midPoint.y;
            if((lines[i][0] - lines[i][2])!= 0){
                leftSlope += - (lines[i][1] - lines[i][3])/(lines[i][0] - lines[i][2]);
            }
            leftCount++;
        }
        else
        {
            rightCenter.x += midPoint.x;
            rightCenter.y += midPoint.y;
            if((lines[i][0] - lines[i][2])!= 0){
                rightSlope += - (lines[i][1] - lines[i][3])/(lines[i][0] - lines[i][2]);
            }
            rightCount++;
        }
    }

    leftCenter.x /= leftCount;
    leftCenter.y /= leftCount;
    leftSlope /= leftCount;

    rightCenter.x /= rightCount;
    rightCenter.y /= rightCount;
    rightSlope /= rightCount;

    std::cout<<"Left Center x: "<<leftCenter.x<<" "<<" Right Center x: "<<rightCenter.x;
    std::cout<<" LeftCount: "<<leftCount<<" RightCount: "<<rightCount<<std::endl;;

    cv::Point target, proj;
    proj.x = (bot_x + m*(bot_y - center_point.y) + m*m*center_point.x)/(1 + m*m); // Verified
    // Earlier: proj.y = m*(bot_x + m*(bot_y - center_point.y) - center_point.x)/(1+ m*m) + center_point.y;
    proj.y = (center_point.y + m*(bot_x - center_point.x) + m*m*bot_y)/(1 + m*m); // Verify it
    // The equation is correct but I think this form is better

    target.x = proj.x + cos(center_angle)*step_move;
    target.y = proj.y + sin(center_angle)*step_move;

    cv::Point point2;
    point2.x = center_point.x - 100;
    point2.y = center_point.y - m*100;

    std::cout<<"target.x: "<<target.x<<" target.y: "<<target.y<<std::endl;
    std::cout<<"proj.x: "<<proj.x<<" "<<"proj.y: "<<proj.y<<std::endl;

    cv::line(img, proj, target, cv::Scalar(150),2,8);
    cv::line(img, cv::Point(bot_x, bot_y),target,cv::Scalar(255), 2, 8);
    cv::namedWindow("Center_path", cv::WINDOW_AUTOSIZE);
    cv::imshow("Center_path", img);
    cv::waitKey(0);
}


int main()
{
    cv::Mat img;
    img=cv::imread("./GetLaneBinary.jpg",0);
    lanesep(img);
    return 0;
}
