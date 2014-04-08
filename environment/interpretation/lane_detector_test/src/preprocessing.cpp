#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cassert>
#include "laneDetector.hpp"

cv::Mat HistogramEqualizationColorImage(cv::Mat src);
cv::Mat RemoveChannel(cv::Mat src, const int channel_id);

cv::Mat LaneDetector::Preprocessing(cv::Mat &image) {

    // Histogram equalization
    return HistogramEqualizationColorImage(image);
}


// Accessory Functions
cv::Mat HistogramEqualizationColorImage(cv::Mat src){
    cv::Mat channel[3];
    cv::split(src, channel);

    cv::equalizeHist(channel[0], channel[0]);
    cv::equalizeHist(channel[1], channel[1]);
    cv::equalizeHist(channel[2], channel[2]);

    cv::merge(channel, 3, src);

    return src;
}

cv::Mat RemoveChannel(cv::Mat src, const int channel_id){
    cv::Mat channel[3];
    cv::split(src, channel);

    assert(channel_id == 0 || channel_id == 1 || channel_id == 2);
    channel[channel_id] = cv::Mat::zeros(src.rows, src.cols, CV_8UC1);

    cv::merge(channel, 3, src);

    return src;

}