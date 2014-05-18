#include "LidarData.h"
#include <cvblob.h>

/*  Filter:
 *  0: No filter
 *  1: Blob filter
 */

#define FILTER 1
#define DEBUG 0


#define CENTERX 500
#define CENTERY 100
#define HOKUYO_SCALE 100
#define RADIUS 30
#define EXPAND_ITER 60
#define intensity(img,i,j,n) *(uchar*)(img->imageData + img->widthStep*i + j*img->nChannels + n) 
#define IMGDATA(image,i,j,k) (((uchar *)image->imageData)[(i)*(image->widthStep) + (j)*(image->nChannels) + (k)])
#define IMGDATAG(image,i,j) (((uchar *)image->imageData)[(i)*(image->widthStep) + (j)])

void LidarData::update_map(const sensor_msgs::LaserScan& scan) {
cv::Mat showImg1;
cv::Mat showImg2;
cv::Mat showImg3;
int min_dist=0,max_dist=500;

    if (DEBUG){
        cv::namedWindow("Control Box", 1);
    }
    
    int minblob_lidar = 150, s=5;
    fstream file;
    file.open("../src/Modules/Lidar/lidar_parameters.txt", ios::in);
    file>>minblob_lidar>>s;
    file.close();

       
    IplImage  *nblobs, *nblobs1, *labelImg;
    cv::Mat img(1000,1000,CV_8UC1, cvScalarAll(0));

    
    cvb::CvBlobs blobs; 

    
    size_t size = scan.ranges.size();
    float angle = scan.angle_min;
    float maxRangeForContainer = scan.range_max - 0.1f;

    for (size_t i = 0; i < size; ++i) {
        float dist = scan.ranges[i];
        if ((dist > scan.range_min) && (dist < maxRangeForContainer)) {
            double x1 = -1 * sin(angle) * dist;
            double y1 = cos(angle) * dist;
            int x = (int) ((x1 * 100) + CENTERX);
            int y = (int) ((y1 * 100) + CENTERY + 30);

            if (x >= 0 && y >= min_dist && (int) x < 1000 && (int) y + 30 + 1 < max_dist) {
                int x2 = x;
                int y2 = (1000 - y - 30 - 1);
				if(x2<0 || x2>1000)
					std::cout<<"load in x:"<<x2<<std::endl;
				if(y2<0 || y2>1000)
					std::cout<<"load in y:"<<y2<<std::endl;
                if (y2>0 && y2<1000 && x2>0 && x2<1000)
					img.at<uchar>(y2,x2)=255;
                }
        }
        angle += scan.angle_increment;
    }

	if (DEBUG) {
                cv::namedWindow("Raw Scan", 0);
                cv::resize(img, showImg1,cv::Size(400,400));  
                cv::imshow("Raw Scan", img);
                cv::waitKey(10);
            }
    
	int erosion_size = 1;  
    switch (FILTER) {
        case 0:
        {
            break;
        }
        case 1:
        {
            
			
			
            erosion_size=3;
            cv::Mat ker1 = cv::getStructuringElement(cv::MORPH_CROSS,
              cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
              cv::Point(3, 3) );  
              
            
            cv::dilate(img, img, ker1);
           
            cv::namedWindow("result",1);
            
            
           
            IplImage img_c=img;
           
            IplImage *labelImg = cvCreateImage( cvGetSize(&img_c), IPL_DEPTH_LABEL, 1 );
            unsigned int result = cvLabel(&img_c, labelImg, blobs);
            IplImage *imgOut = cvCreateImage(cvGetSize(&img_c), IPL_DEPTH_8U, 3);
            cvRenderBlobs(labelImg, blobs, &img_c, imgOut, CV_BLOB_RENDER_COLOR);
            cvFilterByArea(blobs, minblob_lidar, img.cols * img.rows);
            cvRenderBlobs(labelImg, blobs, &img_c, imgOut, CV_BLOB_RENDER_COLOR);
            cv::Mat img_f(imgOut);
			
          
            cv::cvtColor(img_f, img_f, CV_BGR2GRAY);
          
            cv::threshold(img_f, img_f, 125, 255, CV_THRESH_BINARY);
			cvReleaseImage(&labelImg);
            cvReleaseImage(&imgOut);
            cvReleaseBlobs(blobs);
			

            if (DEBUG) {
                cv::namedWindow("Blob Filter", 0);
                cv::resize(img, showImg2,cv::Size(400,400));
                cv::imshow("Blob Filter", showImg2);
                cv::waitKey(10);
            }
            break;
        }
    }

    erosion_size = 1;  
			cv::Mat ker1 = cv::getStructuringElement(cv::MORPH_CROSS,
              cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
              cv::Point(1, 1) );

    cv::dilate(img, img, ker1);
    

    if (DEBUG) {
        cv::namedWindow("Dilate Filter", 0);
        cv::resize(img, showImg3,cv::Size(400,400));
        cv::imshow("Dilate Filter", showImg3);
        cv::waitKey(10);
         }

    publishData(img);
}

void LidarData::writeVal(int val){
    fstream file;
    file.open("../src/Modules/Lidar/lidar_parameters.txt", ios::out);
    file<<cvGetTrackbarPos("minblob_lidar", "Control Box")<<endl;
    file<<cvGetTrackbarPos("kernel 1", "Control Box")<<endl;
    file.close();
}

LidarData::~LidarData() {
}

LidarData::LidarData() {
}

void LidarData::publishData(cv::Mat img) {
    cv_bridge::CvImage out_msg;
    out_msg.encoding = sensor_msgs::image_encodings::MONO8;
    out_msg.image = img;
    out_msg.encoding = sensor_msgs::image_encodings::MONO8;
    obstacle_publisher.publish(out_msg.toImageMsg());
}

