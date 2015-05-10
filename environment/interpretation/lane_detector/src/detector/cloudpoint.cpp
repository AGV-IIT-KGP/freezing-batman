#include <opencv2/highgui/highgui.hpp>
#include <ros/package.h>
#include <laneDetector.hpp>
#include <math.h>

#define ANGLE_SPREAD 180
#define BOT_REFERENCE_X 500
#define BOT_REFERENCE_Y 100   //100 pixels with respect to cartesian coordinates 
#define LARGE_VAL 10000     

pcl::PointCloud<pcl::PointXYZ>::Ptr LaneDetector::generatecloud(cv::Mat& img)
{
	typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
	PointCloud::Ptr cloud_msg(new PointCloud);
	cloud_msg->height=1;
	cloud_msg->width=ANGLE_SPREAD;
	for(int i=0;i<ANGLE_SPREAD;i++)
	{
		int c=0;
		double x=BOT_REFERENCE_X,y=BOT_REFERENCE_Y;
		while(x>=0 && x<img.rows && y>0 && y<=img.cols)
		{
			if(img.at<uchar>(int(img.rows-y),int(x)) >200)
			{
				c++;
				cloud_msg->points.push_back(pcl::PointXYZ((int)x,(int)y,0));
				break;
			}
			x-=cos(i*CV_PI/180);
			y+=sin(i*CV_PI/180);
		}
		if(c==0)
		{
			/*cloud.points[i].x=LARGE_VAL;
			cloud.points[i].y=LARGE_VAL;
			cloud.points[i].z=0;*/
			cloud_msg->points.push_back(pcl::PointXYZ(LARGE_VAL,LARGE_VAL,0));
		}
	}
	//cloud_pub.publish(cloud);
	return cloud_msg;
}