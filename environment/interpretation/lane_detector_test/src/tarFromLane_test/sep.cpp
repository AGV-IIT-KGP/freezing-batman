
#include <ros/ros.h>
#include <math.h>
#include <string>
#include <opencv/cv.h>
#include <opencv/highgui.h>

const int bot_x=320, bot_y=480;
const int step_move= 400;

void lanesep(cv::Mat img)
{
  cv::Mat cdst;
  cv::namedWindow("Original", cv::WINDOW_AUTOSIZE);
  std::vector<cv::Vec4i>lines;
  cv::Point center_point;
  center_point.x=0, center_point.y=0;
  double center_angle=0.0;  
  cv::HoughLinesP(img,lines,1, CV_PI/1000, 5, 20,5);
  for( int i = 0; i < lines.size(); i++ )
  {
     cv::Vec4i l = lines[i];
     line( img, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255,255,255), 3, CV_AA);
   }
  cv::imshow("Original",img);
  cv::waitKey(0);
 for(int i=0;i<lines.size(); i++)
  {
    cv::Vec4i p=lines[i];
    center_point.x += (p[0]+p[2])/2;
    center_point.y += (p[1]+p[3])/2;
    // center_angle+= std::atan2((p[0] - p[3]), (p[1] - p[3]));
    if((p[1]-p[3])!=0){
    double r=std::atan2((p[1] - p[3]), (p[0] - p[2]));
    // std::cout<<r<<"  "<<std::endl;
    center_angle-= fabs(r);
    if(fabs(r)<1)std::cout<<fabs(r)<<std::endl;}
  }
  std::cout<<center_angle<<std::endl;
  center_point.x= center_point.x/lines.size();
  center_point.y= center_point.y/lines.size();
  std::cout<<center_point.x<<" "<<center_point.y<<std::endl;
  center_angle= center_angle/lines.size();
  std::cout<<center_angle<<std::endl;
  double m = (center_angle != 0) ? tan(center_angle) : 100000; 
  std::cout<<m<<"slope"<<std::endl;
  cv::Point leftCenter(0,0), rightCenter(0, 0); 
  
  double leftSlope = 0.0, rightSlope = 0.0, leftCount = 0, rightCount = 0; 
  for( int i = 1; i < lines.size(); i++ ) 
    { 
       cv::Vec4i p= lines[i];
       cv::Point midPoint = cv::Point((p[0] + p[2])/2, (p[1] + p[3])/2); 
       double L11 = (0-center_point.y + m*(0-center_point.x + 0.0))/m; 
       double L22 = (midPoint.y-center_point.y + m*(midPoint.x-center_point.x + 0.0))/m; 
            if(L11*L22 > 0) 
            { 
                leftCenter.x += midPoint.x; 
                leftCenter.y += midPoint.y;
                if((lines[i][0] - lines[i][2])!= 0) 
                  leftSlope += -(lines[i][1] - lines[i][3])/(lines[i][0] - lines[i][2]); 
                //else
                 // leftSlope+= -100000;
                leftCount++; 
            } 
            else 
            { 
                rightCenter.x += midPoint.x; 
                rightCenter.y += midPoint.y; 
                if((lines[i][0] - lines[i][2])!= 0)
                  rightSlope += -(lines[i][1] - lines[i][3])/(lines[i][0] - lines[i][2]); 
                //else
                  //rightSlope+= -100000;
                rightCount++; 
            } 
        } 
        leftCenter.x /= leftCount;        leftCenter.y /= leftCount;        leftSlope /= leftCount; 
        rightCenter.x /= rightCount;    rightCenter.y /= rightCount;    rightSlope /= rightCount; 
        std::cout<<leftCenter.x<<" "<<rightCenter.x<<" "<<leftCount<<" "<<rightCount<<std::endl;;
        cv::Point target, proj;
        proj.x=(bot_x + m*(bot_y - center_point.y) + m*m*center_point.x)/(1+ m*m);
        proj.y= m*(bot_x + m*(bot_y- center_point.y) - center_point.x)/(1+ m*m) + center_point.y;
        target.x= proj.x + cos(center_angle)* step_move;
        target.y= proj.y + sin(center_angle)* step_move;
        cv::Point point2;
        point2.x= center_point.x - 100;
        point2.y= center_point.y - m*100;
        // std::cout<<m<<std::endl;
        std::cout<<target.x<<" "<<target.y<<std::endl;
        std::cout<<proj.x<<" "<<proj.y<<std::endl;

        cv::line(img,proj, target, cv::Scalar(150),2,8);
        cv::line(img, center_point,point2, cv::Scalar(255), 2, 8);
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