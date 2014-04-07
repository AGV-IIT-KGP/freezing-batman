/* 
 * File:   LaneDetector.cpp
 * Author: samuel
 * 
 * Created on 14 December, 2013, 3:36 AM
 */

#include "LaneDetector.hpp"

void LaneDetector::interpret() {

}

LaneDetector::LaneDetector() {
    canny_kernel = 3;
    high_threshold = 900;
    low_threshold = 550;
    vote = 25;
    length = 50;
    mrg = 10;
    k = 90;
    warp_matrix = cvCreateMat(3, 3, CV_32FC1);
    mouseParam = 5;
}

LaneDetector::LaneDetector(const LaneDetector& orig) {
}

LaneDetector::~LaneDetector() {
}

IplImage* LaneDetector::colorBasedLaneDetection(IplImage *frame, int k) {
    IplImage *frame_out = cvCreateImage(cvGetSize(frame), frame->depth, 1);
    int height = gray_frame->height;
    int width = gray_frame->width;
    uchar *data;

    cvGetRawData(gray_frame, (uchar**) & data);

    //Calculate the mean of the image
    double total = 0;
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            total += data[i * gray_frame->width + j];
        }
    }
    mean = (total / (height * width));

    //Calculate the standard deviation of the image
    double var = 0;
    for (int a = 0; a < height; a++) {
        for (int b = 0; b < width; b++) {
            var += ((data[a * gray_frame->widthStep + b] - mean) * (data[a * gray_frame->widthStep + b] - mean));
        }
    }
    var /= (height * width);
    std_dev = sqrt(var);
    cvThreshold(gray_frame, frame_out, (mean + k / 100.0 * std_dev), 255, CV_THRESH_BINARY);
    return frame_out;
}

void LaneDetector::applyHoughTransform(IplImage* img, IplImage *dst, int vote, int length, int mrgh) {
    CvSeq* lines;
    CvMemStorage* storage = cvCreateMemStorage(0);
    cvSetZero(dst);
    int i;

    lines = cvHoughLines2(img, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI / 180, vote, length, mrgh);
    int n = lines->total;
    for (i = 0; i < n; i++) {
        CvPoint* line = (CvPoint*) cvGetSeqElem(lines, i);
        cvLine(dst, line[0], line[1], CV_RGB(255, 255, 255), 15, 8);
    }

    cvReleaseMemStorage(&storage);
}

CvSeq* LaneDetector::GetHoughLanes(IplImage* img, int vote, int length, int mrgha) {
    CvSeq* lines;
    CvMemStorage* storage = cvCreateMemStorage(0);
    lines = cvHoughLines2(img, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI / 180, vote, length, mrgha);

    return lines;
}

void LaneDetector::initializeLaneVariables(int argc, char** argv, ros::NodeHandle nh) {

    it = new image_transport::ImageTransport(nh);
    
    offset = cvPoint((N - 1) / 2, (N - 1) / 2);
    gray_frame = cvCreateImage(size, depth, 1);
    kernel_frame = cvCreateImage(cvSize(gray_frame->width + N - 1, gray_frame->height + N - 1), gray_frame->depth, gray_frame->nChannels);
    edge_frame = cvCreateImage(cvGetSize(kernel_frame), IPL_DEPTH_8U, kernel_frame->nChannels);
    gray_hough_frame = cvCreateImage(cvGetSize(kernel_frame), IPL_DEPTH_8U, kernel_frame->nChannels);
    warp_img = cvCreateImage(cvSize(MAP_MAX, MAP_MAX), 8, 1);
    ker1 = cvCreateStructuringElementEx(3, 3, 1, 1, CV_SHAPE_ELLIPSE);

    if (DEBUG) {
        cvNamedWindow("Control Box", 1);
        cvNamedWindow("warp", 0);
        cvNamedWindow("view", 0);
        cvNamedWindow("view_orig", 0);
        cvNamedWindow("Debug", 0);
    }

    cvCreateTrackbar("K", "Control Box", &k, 300, NULL);
    cvCreateTrackbar("Vote", "Control Box", &vote, 50, NULL);
    cvCreateTrackbar("Length", "Control Box", &length, 100, NULL);
    cvCreateTrackbar("Merge", "Control Box", &mrg, 15, NULL);
    cvCreateTrackbar("Canny Kernel", "Control Box", &canny_kernel, 10, NULL);
    cvCreateTrackbar("High Canny Threshold", "Control Box", &high_threshold, 2000, NULL);
    cvCreateTrackbar("Low Canny Threshold", "Control Box", &low_threshold, 2000, NULL);

    //Destination variables
    int widthInCM = 100, h1 = 2, h2 = 75; //width and height of the lane. width:widthoflane/scale;

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

    cvGetPerspectiveTransform(dstQuad, srcQuad, warp_matrix);
}

IplImage* LaneDetector::getLaneLines(IplImage* src) {
    IplImage* dst = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 1);
    IplImage* color_dst = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 3);
    CvMemStorage* storage = cvCreateMemStorage(0);
    CvSeq* lines = 0;
    int i;
    cvCanny(src, dst, 50, 200, 3);

    color_dst = cvCreateImage(cvGetSize(dst), dst->depth, 3);
    cvSetZero(color_dst);
    //cvCvtColor( dst, color_dst, CV_GRAY2BGR );

    lines = cvHoughLines2(dst, storage, CV_HOUGH_STANDARD, 1, CV_PI / 180, 60, 0, 0);

    for (i = 0; i < MIN(lines->total, 100); i++) {
        float* line = (float*) cvGetSeqElem(lines, i);
        float rho = line[0];
        float theta = line[1];
        CvPoint pt1, pt2;

        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000 * (-b));
        pt1.y = cvRound(y0 + 1000 * (a));
        pt2.x = cvRound(x0 - 1000 * (-b));
        pt2.y = cvRound(y0 - 1000 * (a));
        cvLine(color_dst, pt1, pt2, CV_RGB(255, 255, 255), 3, 8);
    }

    cvCvtColor(color_dst, dst, CV_BGR2GRAY);
    return dst;
}

int min(int value) {
    if (value > 999) {
        return 999;
    }
    if (value < 0) {
        return 0;
    }
}


void mouseHandler(int event, int x, int y, int flags, void* param) {

    if (event == CV_EVENT_RBUTTONDOWN) {
        std::cout << "@ Right mouse button pressed at: " << x << "," << y << std::endl;
    }
}

void LaneDetector::getLanes(const sensor_msgs::ImageConstPtr& image) {
    try {
        img = bridge.imgMsgToCv(image, "bgr8");
        cvWaitKey(WAIT_TIME);
    } catch (sensor_msgs::CvBridgeException& e) {
        ROS_ERROR("ERROR IN CONVERTING IMAGE!!!");
    }
    size = cvGetSize(img);
    depth = img->depth;
    cvCvtColor(img, gray_frame, CV_BGR2GRAY); // converting image to gray scale
    if (choice == 1) {
        lane = colorBasedLaneDetection(gray_frame, k);
    }
    if ((choice == 2) || (choice == 0)) {
        cvEqualizeHist(gray_frame, gray_frame);
        //cvCopyMakeBorder(gray_frame, kernel_frame, 
        cvSmooth(gray_frame, gray_frame, CV_GAUSSIAN);
        // canny edge detection
        cvCopyMakeBorder(gray_frame, kernel_frame, offset, IPL_BORDER_REPLICATE, cvScalarAll(0));
        cvCanny(kernel_frame, edge_frame, low_threshold, high_threshold);
        applyHoughTransform(edge_frame, gray_hough_frame, vote, length, mrg);
        //TODO: Truncate the image
        if (DEBUG) {
            cvResize(gray_hough_frame, show_img1);
            cvShowImage("Hough", show_img1);
            cvWaitKey(WAIT_TIME);
        }
    }
    if (choice == 0) {
        //TODO : reduce the kernel size and copy it to the lane
        lane = cvCreateImage(cvGetSize(gray_hough_frame), 8, gray_hough_frame->nChannels);
        lane = gray_hough_frame;
    }
    if (choice == 2) {
        lane = joinResult(colorBasedLaneDetection(img, k), gray_hough_frame);
    }

    if (DEBUG) {
        cvSetMouseCallback("view", &mouseHandler, 0);
        cvResize(lane, show_img2);

        cvShowImage("view", lane);
        cvWaitKey(WAIT_TIME);
    }

    cvWarpPerspective(lane, warp_img, warp_matrix, CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS);

    if (DEBUG) {
        cvResize(warp_img, show_img3);
        cvShowImage("warp", show_img3);
        cvWaitKey(WAIT_TIME);
    }
    cvDilate(warp_img, warp_img, ker1, 55);

    publishLanes(warp_img);
    cvReleaseImage(&lane);
}

void LaneDetector::publishLanes(IplImage* final_img) {

    //image_transport::ImageTransport it(*lane_node);
    image_transport::Publisher pub = it->advertise("camera/image", 10);

}

IplImage* LaneDetector::joinResult(IplImage* color_gray, IplImage* hough_gray) {
    int i, j;
    int index_color;
    int index_hough;
    IplImage* lane_gray = cvCreateImage(cvGetSize(color_gray), color_gray->depth, 1);
    uchar* color_data = (uchar*) color_gray->imageData;
    uchar* hough_data = (uchar*) hough_gray->imageData;
    uchar* lane_data = (uchar*) lane_gray->imageData;

    for (i = 0; i < color_gray->height; i++) {
        for (j = 0; j < color_gray->width; j++) {
            index_color = i * color_gray->widthStep + j;
            //           index_hough = i+((N-1)/2) * hough_gray->widthStep + j;
            index_hough = i * hough_gray->widthStep + j;
            if ((color_data[index_color] > 0) && (hough_data[index_hough] > 0)) {
                lane_data[index_color] = 255;
            } else {
                lane_data[index_color] = 0;
            }
        }
    }

    return lane_gray;
}

image_transport::ImageTransport *LaneDetector::getLaneNode() {
    return it;
}

int main(int argc, char** argv) {
    LaneDetector lane_detector;
    ros::init(argc, argv, "hvhg");
    ros::NodeHandle nh("hvhg");
    //lane_detector.initializeLaneVariables(argc, argv, nh);
    image_transport::Subscriber sub = lane_detector.getLaneNode()->subscribe("camera/image", 2, &LaneDetector::getLanes, &lane_detector);
    ros::Rate loop_rate(LOOP_RATE);
    ROS_INFO("LANE_THREAD STARTED");
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("Lane code exiting");
    return 0;
}
