#include <cassert>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <laneDetector.hpp>

cv::Mat histogramEqualizationColorImage(cv::Mat src);
cv::Mat removeChannel(cv::Mat src, const int channel_id);

cv::Mat LaneDetector::preprocessing(cv::Mat &image) {
    cv::Mat result;

    cv::GaussianBlur(image, result, cv::Size(5, 5), 15, 15);

    // Histogram equalization
    return histogramEqualizationColorImage(image);
}

// Accessory Functions

cv::Mat histogramEqualizationColorImage(cv::Mat src) {
    cv::Mat channel[3];
    cv::split(src, channel);

    cv::equalizeHist(channel[0], channel[0]);
    cv::equalizeHist(channel[1], channel[1]);
    cv::equalizeHist(channel[2], channel[2]);

    cv::merge(channel, 3, src);

    return src;
}

cv::Mat removeChannel(cv::Mat src, const int channel_id) {
    cv::Mat channel[3];
    cv::split(src, channel);

    assert(channel_id == 0 || channel_id == 1 || channel_id == 2);
    channel[channel_id] = cv::Mat::zeros(src.rows, src.cols, CV_8UC1);

    cv::merge(channel, 3, src);

    return src;
}
