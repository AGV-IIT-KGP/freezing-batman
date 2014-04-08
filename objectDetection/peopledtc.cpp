#include <iostream>
#include <opencv2/opencv.hpp>

int main (int argc, const char *argv[]){
    cv::VideoCapture cap(CV_CAP_ANY);
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
}
