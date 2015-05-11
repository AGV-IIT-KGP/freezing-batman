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
    cloud_msg->header.frame_id="map";
	cloud_msg->height=1;
	cloud_msg->width=ANGLE_SPREAD;
	for(int i=0;i<ANGLE_SPREAD;i++)
	{
		int c=0;
		double x=BOT_REFERENCE_X,y=BOT_REFERENCE_Y;
		//cloud_msg->header.stamp = ros::Time::now ();
		while(x>=0 && x<img.cols && y>0 && y<=img.rows)
		{
			if(img.at<uchar>(int(img.rows-y),int(x)) >200)
			{
				c++;
				
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

	/*for(int i=0;i<img.rows;i++)
	{
		for(int j=0;j<img.cols;j++)
		{
			
			cloud_msg->points.push_back(pcl::PointXYZ(j,(img.rows-i),0));
		}
	}*/

	//cloud_pub.publish(cloud);
	return cloud_msg;
}

