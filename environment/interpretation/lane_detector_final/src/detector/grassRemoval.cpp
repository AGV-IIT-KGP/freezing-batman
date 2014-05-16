#include <laneDetector.hpp>

cv::Mat LaneDetector::grassRemoval(cv::Mat &image) {
    svm->predictKernelWise(image, kernel_size);

    return image;
}
