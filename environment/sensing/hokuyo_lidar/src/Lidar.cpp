/* 
 * File:   HokuyoLidar.cpp
 * Author: satya
 * 
 * Created on December 12, 2013, 9:47 PM
 */

#include "LifeCycle.hpp"
#include "Lidar.hpp"
#include <cvblob.h>

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

HokuyoLidar::HokuyoLidar() {
}

HokuyoLidar::HokuyoLidar(const HokuyoLidar& orig) {
}

LogitechCamera::LogitechCamera(int argc, char** argv) : Sensor(argc, argv) {
    initializeParameters(argc, argv);

    ros::init(argc, argv, node_name.c_str());

    setupCommunications();
}

LogitechCamera::~LogitechCamera() {
}

void LidarData::update_map(const sensor_msgs::LaserScan& scan) {

    //TODO: Fusion needs to be implemented in the STRATEGY module


    pthread_mutex_lock(&lidar_map_mutex);
    for (int i = 0; i < MAP_MAX; i++) {
        for (int j = 0; j < MAP_MAX; j++) {
            lidar_map[i][j] = IMGDATA(img, MAP_MAX - j - 1, i, 0);
        }
    }
    pthread_mutex_unlock(&lidar_map_mutex);
    cvReleaseImage(&img);
}

void LogitechCamera::initializeParameters() {
    lidar_id = 0;
    node_name = std::string("HokuyoLidar");
    topic_name = std::string("sensors/HokuyoLidar");
    message_queue_size = 10;
}

void LogitechCamera::initializeParameters(int argc, char** argv) {
    lidar_id = std::atoi(argv[1]);
    node_name = std::string("sensors_HokuyoLidar_") + std::string(argv[1]);
    topic_name = std::string("sensors/HokuyoLidar/") + std::string(argv[1]);
    message_queue_size = 10;


    //initialize variables
    if (DEBUG){
        cvNamedWindow("Control Box", 1);
    }
    
    int minblob_lidar = 150, s=5;
    fstream file;
    file.open("../src/Modules/Lidar/lidar_parameters.txt", ios::in);
    file>>minblob_lidar>>s;
    file.close();

    cvCreateTrackbar("minblob_lidar", "Control Box", &minblob_lidar, 1000, &writeVal);
    cvCreateTrackbar("kernel 1", "Control Box", &s, 20, &writeVal);
        
    IplImage *img, *nblobs, *nblobs1, *labelImg;
    img = cvCreateImage(cvSize(MAP_MAX, MAP_MAX), 8, 1);
    cvSet(img, cvScalar(0));

    //TODO: put kernel initialization in constructor
    IplConvKernel *ker1, *ker2;
    cvb::CvBlobs blobs;
    uchar * ptr;

    //initialize variables ended
}

bool HokuyoLidar::connect() {
  //Taking data from hokuyo node

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

            if (x >= 0 && y >= 0 && (int) x < MAP_MAX && (int) y < MAP_MAX) {
                int x2 = (x);
                int y2 = (MAP_MAX - y - 30 - 1);

                ptr = (uchar *) (img->imageData + y2 * img->widthStep);
                ptr[x2] = 255;
            }
        }
        angle += scan.angle_increment;
    }

	if (DEBUG) {
                cvNamedWindow("Raw Scan", 0);
                cvResize(img, showImg1);
                cvShowImage("Raw Scan", img);
                cvWaitKey(WAIT_TIME);
            }
    //Filtering
}

bool HokuyoLidar::disconnect() {

}

bool HokuyoLidar::fetch() {


    switch (FILTER) {
        case 0:
        {
            break;
        }
        case 1:
        {
            labelImg = cvCreateImage(cvSize(MAP_MAX, MAP_MAX), IPL_DEPTH_LABEL, 1);
            nblobs = cvCreateImage(cvSize(MAP_MAX, MAP_MAX), 8, 3);
            nblobs1 = cvCreateImage(cvSize(MAP_MAX, MAP_MAX), 8, 3);
            cvSet(labelImg, cvScalar(0));

            ker1 = cvCreateStructuringElementEx(s, s, 2, 2, CV_SHAPE_ELLIPSE);
            ker2 = cvCreateStructuringElementEx(7, 7, 3, 3, CV_SHAPE_ELLIPSE);
            cvDilate(img, img, ker1, 1);
            //cvErode(filtered_img,filtered_img,ker2,1);
            unsigned int result = cvLabel(img, labelImg, blobs);
            cvRenderBlobs(labelImg, blobs, nblobs, nblobs, CV_BLOB_RENDER_COLOR);
            cvFilterByArea(blobs, minblob_lidar, img->height * img->width);
            cvRenderBlobs(labelImg, blobs, nblobs1, nblobs1, CV_BLOB_RENDER_COLOR);
            //converts nblobs1 to filtered_img(grayscale)
            cvCvtColor(nblobs1, img, CV_RGB2GRAY);
            //thresholds the filtered_img based on threshold value
            cvThreshold(img, img, 125, 255, CV_THRESH_BINARY);

            cvReleaseImage(&labelImg);
            cvReleaseImage(&nblobs);
            cvReleaseImage(&nblobs1);
            cvReleaseBlobs(blobs);

            cvReleaseStructuringElement(&ker1);
            cvReleaseStructuringElement(&ker2);

            if (DEBUG) {
                cvNamedWindow("Blob Filter", 0);
                cvResize(img, showImg2);
                cvShowImage("Blob Filter", showImg2);
                cvWaitKey(WAIT_TIME);
            }
            break;
        }
    }
    
  
    ker1 = cvCreateStructuringElementEx(3, 3, 1, 1, CV_SHAPE_ELLIPSE);
    cvDilate(img, img, ker1, EXPAND_ITER);
    cvReleaseStructuringElement(&ker1);

    if (DEBUG) {
        cvNamedWindow("Dilate Filter", 0);
        cvResize(img, showImg3);
        cvShowImage("Dilate Filter", showImg3);
        cvWaitKey(WAIT_TIME);
    }


}

int main() {
    int a = 5;
    return 0;
}
