#include "LaneDetector.hpp"
#include <cvblob.h>
#include <math.h>

using namespace cvb;

#define DEBUG 1
#define EXPANSION 10

sensor_msgs::CvBridge bridge;
static int iter = 0;
IplImage *blue_img;
IplImage *green_img;
IplImage *filter_img;
IplImage *warp_img;
IplImage* img;
IplConvKernel *ker1;
CvPoint2D32f srcQuad[4], dstQuad[4];
CvMat* warp_matrix = cvCreateMat(3, 3, CV_32FC1);

int vote = 16, length = 30, mrg = 100;
int k = 200;

LaneDetector::LaneDetector(ros::NodeHandle node_handle) {
    loadParams(node_handle);
    setupComms(node_handle);
}

void LaneDetector::setupComms(ros::NodeHandle node_handle) {
    image_transport::ImageTransport image_transport(node_handle);
    lanes_publisher = image_transport.advertise(published_topic_name, 2);
    image_subscriber = image_transport.subscribe(subscribed_topic_name, 2, &LaneDetector::markLane, this);
    std::cout << "Communications started with : " << std::endl;
    std::cout << "\tSubscriber topic : " << subscribed_topic_name << std::endl;
    std::cout << "\tPublisher topic  : " << published_topic_name << std::endl;
}

void LaneDetector::loadParams(ros::NodeHandle node_handle) {
    published_topic_name = std::string("/lane_detector/lanes");
    subscribed_topic_name = std::string("/logitech_camera/image");
}

IplImage* LaneDetector::colorBasedLaneDetection(IplImage *img) {
    double mean, std_dev;
    int height = img->height;
    int width = img->width;
    //Calculating Mean of the image
    double total = 0;
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            total += CV_IMAGE_ELEM(img, unsigned char, i, j);
        }
    }
    mean = (total / (height * width));
    //Calculating Standard Deviation of the image
    double var = 0;
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            var += pow(CV_IMAGE_ELEM(img, unsigned char, i, j) - mean, 2);
        }
    }
    var /= (height * width);
    std_dev = sqrt(var);
    cvThreshold(img, img, mean + ((k * std_dev) / 100.0), 255, CV_THRESH_BINARY);
    //Morphological Operation to reduce noise
    IplConvKernel *convKernel = cvCreateStructuringElementEx(5, 5, 2, 2, CV_SHAPE_RECT);
    IplImage* simg = cvCloneImage(img);
    cvMorphologyEx(img, simg, NULL, convKernel, CV_MOP_OPEN);
    cvNot(simg, img);
    cvMorphologyEx(img, simg, NULL, convKernel, CV_MOP_OPEN);
    cvNot(simg, img);
    return img;
}

IplImage* LaneDetector::applyHoughTransform(IplImage* img) {
    CvSeq* lines;
    CvMemStorage* storage = cvCreateMemStorage(0);
    int i;
    lines = cvHoughLines2(img, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI / 180, vote, length, mrg);
    cvSetZero(img);
    int n = lines->total;
    for (i = 0; i < n; i++) {
        CvPoint* line = (CvPoint*) cvGetSeqElem(lines, i);
        cvLine(img, line[0], line[1], CV_RGB(255, 255, 255), 5);
    }
    cvReleaseMemStorage(&storage);
    return img;
}

void LaneDetector::initializeLaneVariables(IplImage *input_frame) {
    blue_img = cvCreateImage(cvGetSize(input_frame), input_frame->depth, 1);
    green_img = cvCreateImage(cvGetSize(input_frame), input_frame->depth, 1);
    filter_img = cvCreateImage(cvGetSize(input_frame), input_frame->depth, 1);
    warp_img = cvCreateImage(cvSize(MAP_MAX, MAP_MAX), 8, 1);
    ker1 = cvCreateStructuringElementEx(3, 3, 1, 1, CV_SHAPE_ELLIPSE);
    //Destination variables
    int widthInCM = 100, h1 = 220, h2 = 295; //width and height of the lane. width:widthoflane/scale;

    srcQuad[0].x = (float) 134; //src Top left
    srcQuad[0].y = (float) 166;
    srcQuad[1].x = (float) 432; //src Top right
    srcQuad[1].y = (float) 170;
    srcQuad[2].x = (float) 62; //src Bottom left
    srcQuad[2].y = (float) 362;
    srcQuad[3].x = (float) 488; //src Bot right
    srcQuad[3].y = (float) 354;

    dstQuad[0].x = (float) (500 - widthInCM / (2)); //dst Top left
    dstQuad[0].y = (float) (999 - h2);
    dstQuad[1].x = (float) (500 + widthInCM / (2)); //dst Top right
    dstQuad[1].y = (float) (999 - h2);
    dstQuad[2].x = (float) (500 - widthInCM / (2)); //dst Bottom left
    dstQuad[2].y = (float) (999 - h1);
    dstQuad[3].x = (float) (500 + widthInCM / (2)); //dst Bot right
    dstQuad[3].y = (float) (999 - h1);


    /*srcQuad[0].x = (float) 97; //src Top left
    srcQuad[0].y = (float) 87;
    srcQuad[1].x = (float) 426; //src Top right
    srcQuad[1].y = (float) 82;
    srcQuad[2].x = (float) 36; //src Bottom left
    srcQuad[2].y = (float) 184;
    srcQuad[3].x = (float) 482; //src Bot right
    srcQuad[3].y = (float) 191;
    
    dstQuad[0].x = (float) (434); //dst Top left
    dstQuad[0].y = (float) (630);
    dstQuad[1].x = (float) (566); //dst Top right
    dstQuad[1].y = (float) (630);
    dstQuad[2].x = (float) (434); //dst Bottom left
    dstQuad[2].y = (float) (714);
    dstQuad[3].x = (float) (566); //dst Bot right
    dstQuad[3].y = (float) (714);*/

    cvGetPerspectiveTransform(dstQuad, srcQuad, warp_matrix);
}

void LaneDetector::populateLanes(IplImage *img) {
    cv::Mat image(img);
    cv_bridge::CvImage message;
    message.encoding = sensor_msgs::image_encodings::MONO8;
    message.image = image;
    lanes_publisher.publish(message.toImageMsg());
}

void LaneDetector::markLane(const sensor_msgs::ImageConstPtr& image) {
    try {
        img = bridge.imgMsgToCv(image, "bgr8");
        cvWaitKey(WAIT_TIME);
    } catch (sensor_msgs::CvBridgeException& e) {
        ROS_ERROR("ERROR IN CONVERTING IMAGE!!!");
    }
    //Displaying the original image
    if (DEBUG) {
        cvShowImage("OriginalImage", img);
        cvWaitKey(WAIT_TIME);
    }
    //Initializing the required image variables
    if (iter == 0) {
        initializeLaneVariables(img);
    }
    iter++;
    //Enhancing the lanes
    cvSplit(img, blue_img, green_img, NULL, NULL);
    cvConvertScale(green_img, green_img, 0.5, 0);
    cvSub(blue_img, green_img, filter_img);
    cvConvertScale(filter_img, filter_img, 2, 0);
    //Displying the filtered image
    if(DEBUG)
    {
        cvShowImage("Filtered Image", filter_img);
        cvWaitKey(WAIT_TIME);
    }
    //Adaptive Thresholding
    filter_img = colorBasedLaneDetection(filter_img);
    //Displying thresholded image
    if(DEBUG)
    {
       cvShowImage("Thresholded Image", filter_img);
       cvWaitKey(WAIT_TIME);
    }
    //Finding Hough Lines
    filter_img = applyHoughTransform(filter_img);
    //Displaying hough lanes
    if(DEBUG)
    {
        cvShowImage("Hough Image", filter_img);
        cvWaitKey(WAIT_TIME);
    }
    //Inverse Perspective Transform
    cvWarpPerspective(filter_img, warp_img, warp_matrix, CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS);
    //Displaying Lane Map
    if (DEBUG) {
        cvShowImage("Lane Map", warp_img);
        cvWaitKey(WAIT_TIME);
    }
    cvDilate(warp_img, warp_img, ker1, EXPANSION);
    populateLanes(warp_img);
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "lane_detector");
    ros::NodeHandle node_handle;

    show_img1 = cvCreateImage(cvSize(400, 400), IPL_DEPTH_8U, 1);
    show_img2 = cvCreateImage(cvSize(400, 400), IPL_DEPTH_8U, 1);
    show_img3 = cvCreateImage(cvSize(400, 400), IPL_DEPTH_8U, 1);
    show_img4 = cvCreateImage(cvSize(400, 400), IPL_DEPTH_8U, 3);

    LaneDetector lane_detector(node_handle);

    ros::spin();
    return 0;
}