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
    warp_matrix = cv::Mat(3, 3, CV_32FC1);
    mouseParam = 5;
}

LaneDetector::LaneDetector(const LaneDetector& orig) {
}

LaneDetector::~LaneDetector() {
}

cv::Mat LaneDetector::colorBasedLaneDetection(cv::Mat frame, int k) {
    // cv::Mat frame_out = cvCreateImage(cvGetSize(frame), frame->depth, 1);   //depth??
    cv::Mat frame_out= cv::Mat(frame.rows,frame.cols,CV_MAT_TYPE(frame.type));
    int height = gray_frame.rows;
    int width = gray_frame.cols;
    uchar *data;

    cvGetRawData(gray_frame, (uchar**) & data);  ///?????????????????????????????????????

    //Calculate the mean of the image
    double total = 0;
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            total += data[i * gray_frame.cols + j];
        }
    }
    mean = (total / (rows * cols));

    //Calculate the standard deviation of the image
    double var = 0;
    for (int a = 0; a < height; a++) {
        for (int b = 0; b < width; b++) {
            var += ((data[a * gray_frame.widthStep + b] - mean) * (data[a * gray_frame.widthStep + b] - mean)); //widthstep
        }
    }
    var /= (rows * cols);
    std_dev = sqrt(var);
    cv::threshold(gray_frame, frame_out, (mean + k / 100.0 * std_dev), 255, CV_THRESH_BINARY);
    return frame_out;
}

void LaneDetector::applyHoughTransform(cv::Mat img, cv::Mat dst, int vote, int length, int mrgh) {
    std::vector<cv::Vec4i> lines;
    // CvMemStorage* storage = cvCreateMemStorage(0);
    dst=cv::Scalar(0,0,0);
    int i;

    cv::HoughLinesP(img, lines, 1, CV_PI / 180, vote, length, mrgh);
    int n = lines.size(); 
    for (i = 0; i < n; i++) {
        cv::Vec4i line = lines[i];
        cv::line(dst, cv::Point(line[0], line[1]),cv::Point(line[2], line[3]), CV_RGB(255, 255, 255), 15, 8);
    }

    // cvReleaseMemStorage(&storage);
}

std::vector<cv::vec4i> LaneDetector::GetHoughLanes(cv::Mat img, int vote, int length, int mrgha) {
    std::vector<cv::Vec4i> lines;
    cv::HoughlinesP(img, lines,  1, CV_PI / 180, vote, length, mrgha);

    return lines;
}

void LaneDetector::initializeLaneVariables(int argc, char** argv, ros::NodeHandle nh) {

    it(nh);     //initialize the image_transport from nodeHandle
    ///// ?????????????????????????????????????????????what is N?
    offset = cv::Point((N - 1) / 2, (N - 1) / 2);
    ///// ?????????????????????????????????where is size and depth initiated?
    gray_frame = cv::Mat(size, depth, 1);
    //////////////////////////////////////////????????????????????????
    kernel_frame = cv::Mat(gray_frame.rows + N - 1, gray_frame.cols + N - 1, CV_MAT_TYPE(gray_frame.type));
    edge_frame = cv::Mat(kernel_frame.rows,kernel_frame.cols, CV_MAT_TYPE(kernel_frame.type));
    gray_hough_frame = cv::Mat(kernel_frame.rows,kernel_frame.cols, CV_MAT_TYPE(kernel_frame.type));
    warp_img = cv::Mat(MAP_MAX, MAP_MAX, CV_8UC1);
    // ker1 = cv::getStructuringElement(3, 3, 1, 1, CV_SHAPE_ELLIPSE);
    ker1 = cv::getStructuringElement(CV_SHAPE_ELLIPSE,cv::Size(3,3),cv::Point(1,1));

    if (DEBUG) {
        cv::namedWindow("Control Box", 1);
        cv::namedWindow("warp", 0);
        cv::namedWindow("view", 0);
        cv::namedWindow("view_orig", 0);
        cv::namedWindow("Debug", 0);
    }

    cv::createTrackbar("K", "Control Box", &k, 300, NULL);
    cv::createTrackbar("Vote", "Control Box", &vote, 50, NULL);
    cv::createTrackbar("Length", "Control Box", &length, 100, NULL);
    cv::createTrackbar("Merge", "Control Box", &mrg, 15, NULL);
    cv::createTrackbar("Canny Kernel", "Control Box", &canny_kernel, 10, NULL);
    cv::createTrackbar("High Canny Threshold", "Control Box", &high_threshold, 2000, NULL);
    cv::createTrackbar("Low Canny Threshold", "Control Box", &low_threshold, 2000, NULL);

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

     warp_matrix=cv::getPerspectiveTransform(dstQuad, srcQuad);
}

cv::Mat LaneDetector::getLaneLines(cv::Mat src) {
    cv::Mat dst = cv::Mat(src.rows,src.cols, CV_8UC1);
    cv::Mat color_dst = cv::Mat(src.rows,src.cols, CV_8UC3,cv::Scalar(0,0,0));
    // CvMemStorage* storage = cvCreateMemStorage(0);
    // CvSeq* lines = 0;
    std::vector<cv::Vec4i> lines;
    int i;
    cv::Canny(src, dst, 50, 200, 3);

    // color_dst = cvCreateImage(cvGetSize(dst), dst->depth, 3);
    // cvSetZero(color_dst);

    //cvCvtColor( dst, color_dst, CV_GRAY2BGR );

    // lines = cvHoughLines2(dst, storage, CV_HOUGH_STANDARD, 1, CV_PI / 180, 60, 0, 0);
    cv::HoughLinesP(img, lines, 1, CV_PI / 180, 60, 0, 0);

    /*for (i = 0; i < MIN(lines->total, 100); i++) {
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
    }*/
    int n = lines.size(); 
    for (i = 0; i < n; i++) {
        cv::Vec4i line = lines[i];
        cv::line(color_dst, cv::Point(line[0], line[1]),cv::Point(line[2], line[3]), CV_RGB(255, 255, 255), 3, 8);
    }    

    cv::CvtColor(color_dst, dst, CV_BGR2GRAY);
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
        img = bridge.imgMsgToCv(image, "bgr8");      ////????????? please recheck this part!!!
        cv::waitKey(WAIT_TIME);
    } catch (sensor_msgs::CvBridgeException& e) {
        ROS_ERROR("ERROR IN CONVERTING IMAGE!!!");
    }
    size = cv::Size(img.rows,img.cols);
    depth = CV_MAT_TYPE(img.type);
    cv::CvtColor(img, gray_frame, CV_BGR2GRAY); // converting image to gray scale
    if (choice == 1) {
        lane = colorBasedLaneDetection(gray_frame, k);
    }
    if ((choice == 2) || (choice == 0)) {
        cv::EqualizeHist(gray_frame, gray_frame);
        //cvCopyMakeBorder(gray_frame, kernel_frame, 
        cv::Smooth(gray_frame, gray_frame, CV_GAUSSIAN);
        // canny edge detection
        cv::copyMakeBorder(gray_frame, kernel_frame, offset, IPL_BORDER_REPLICATE, cvScalarAll(0));
          ///////????????????????????????????? what values are to be given for top,bottom,left,right in above line (instead of offset)
        cv::Canny(kernel_frame, edge_frame, low_threshold, high_threshold);
        applyHoughTransform(edge_frame, gray_hough_frame, vote, length, mrg);
        //TODO: Truncate the image
        if (DEBUG) {
            cv::resize(gray_hough_frame, show_img1);
            cv::imshow("Hough", show_img1);
            cv::waitKey(WAIT_TIME);
        }
    }
    if (choice == 0) {
        //TODO : reduce the kernel size and copy it to the lane
        lane = cv::Mat(gray_hough_frame.rows,gray_hough_frame.cols,CV_MAT_TYPE(gray_hough_frame.type));
        lane = gray_hough_frame;
    }
    if (choice == 2) {
        lane = joinResult(colorBasedLaneDetection(img, k), gray_hough_frame);
    }

    if (DEBUG) {
        cv::setMouseCallback("view", &mouseHandler, 0);
        cv::resize(lane, show_img2);

        cv::imshow("view", lane);
        cv::waitKey(WAIT_TIME);
    }
    ///????????????????? Could not understand the parameters used in the following line.
    //cv::warpPerspective(lane, warp_img, warp_matrix, cv::Size(1000, 1000), cv::INTER_NEAREST , cv::BORDER_CONSTANT); will this work?
    cvWarpPerspective(lane, warp_img, warp_matrix, CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS);

    if (DEBUG) {
        cv::resize(warp_img, show_img3);
        cv::imshow("warp", show_img3);
        cv::waitKey(WAIT_TIME);
    }
    cv::dilate(warp_img, warp_img, ker1, 55);

    publishLanes(warp_img);
    // cvReleaseImage(&lane);
}

void LaneDetector::publishLanes(cv::Mat final_img) {

    //image_transport::ImageTransport it(*lane_node);
    image_transport::Publisher pub = it->advertise("camera/image", 10);

}

cv::Mat LaneDetector::joinResult(cv::Mat color_gray, cv::Mat hough_gray) {
    int i, j;
    int index_color;
    int index_hough;
    cv::mat lane_gray = cv::Mat(color_gray.rows,color_gray.cols, CV_MAT_TYPE(color_gray.type));
    // uchar* color_data = (uchar*) color_gray->imageData;
    // uchar* hough_data = (uchar*) hough_gray->imageData;
    // uchar* lane_data = (uchar*) lane_gray->imageData;

    for (i = 0; i < color_gray.rows; i++) {
        for (j = 0; j < color_gray.cols; j++) {
            // index_color = i * color_gray->widthStep + j;
            //           index_hough = i+((N-1)/2) * hough_gray->widthStep + j;
            // index_hough = i * hough_gray->widthStep + j;
            if ((color_data.at<uchar>(i,j) > 0) && (hough_data.at<uchar>(i,j) > 0)) {
                lane_data.at<uchar>(i,j) = 255;
            } else {
                lane_data.at<uchar>(i,j) = 0;
            }
        }
    }

    return lane_gray;
}

image_transport::ImageTransport LaneDetector::getLaneNode() {
    return it;
}

int main(int argc, char** argv) {
    LaneDetector lane_detector;
    ros::init(argc, argv, "hvhg");
    ros::NodeHandle nh("hvhg");
    //lane_detector.initializeLaneVariables(argc, argv, nh);
    image_transport::Subscriber sub = lane_detector.getLaneNode().subscribe("camera/image", 2, &LaneDetector::getLanes, &lane_detector);
    ros::Rate loop_rate(LOOP_RATE);
    ROS_INFO("LANE_THREAD STARTED");
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("Lane code exiting");
    return 0;
}
