#include <laneDetector.hpp>

static int obstacle_removal_dilation_size = 30; //variable used for dilating and eroding.. to be changed only if dimension of image changes.
static int obstacle_removal_hue = 25; //used to remove obstacle, change only after calibration.
static int obstacle_removal_saturation = 100; //used to remove obstacle, change only after calibration.

int rgb2hsv(float &h, float &s, float &v, int r, int g, int b) {
    int rgbMin, rgbMax;

    rgbMin = r < g ? (r < b ? r : b) : (g < b ? g : b);
    rgbMax = r > g ? (r > b ? r : b) : (g > b ? g : b);

    v = rgbMax;
    if (v == 0) {
        h = 0;
        s = 0;
        return -1;
    }
    s = 255 * long(rgbMax - rgbMin) / v;
    if (s == 0) {
        h = 0;
        return -2;
    }

    if (rgbMax == r) {
        h = 0 + 43 * (g - b) / (rgbMax - rgbMin);
    } else if (rgbMax == g) {
        h = 85 + 43 * (b - r) / (rgbMax - rgbMin);
    } else {
        h = 171 + 43 * (r - g) / (rgbMax - rgbMin);
    }

    return 0;
}

cv::Mat ObstacleRemovedBinary(cv::Mat &image) {
    cv::Mat binary_after_HSV_thresholding(image.rows, image.cols, CV_8UC1, cv::Scalar(0, 0, 0));
    float h = 0, s = 0, v = 0;
    for (int i = 0; i < image.rows; i++) {
        for (int j = 0; j < image.cols; j++) {
            if (image.at<cv::Vec3b>(i, j)[0] && image.at<cv::Vec3b>(i, j)[1] && image.at<cv::Vec3b>(i, j)[2]) {
                rgb2hsv(h, s, v, (int) image.at<cv::Vec3b>(i, j)[2], (int) image.at<cv::Vec3b>(i, j)[1], (int) image.at<cv::Vec3b>(i, j)[0]);
                if (h < obstacle_removal_hue && s > obstacle_removal_saturation) {
                    binary_after_HSV_thresholding.at<uchar>(i, j) = 255;
                }
            }
        }
    }
    return binary_after_HSV_thresholding;
}

cv::Mat LaneDetector::obstacleRemoval(cv::Mat &image) {
    cv::Mat img_HSV(image.rows, image.cols, CV_8UC3);
    cv::Mat binary_after_HSV_thresholding(image.rows, image.cols, CV_8UC1);
    cv::Mat binary_dialated(image.rows, image.cols, CV_8UC1);

    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
                                                cv::Size(2 * obstacle_removal_dilation_size + 1, 2 * obstacle_removal_dilation_size + 1),
                                                cv::Point(obstacle_removal_dilation_size, obstacle_removal_dilation_size));
    binary_after_HSV_thresholding = ObstacleRemovedBinary(image);
    cv::dilate(binary_after_HSV_thresholding, binary_dialated, element);
    for (int i = 0; i < binary_dialated.rows; i++) {
        for (int j = 0; j < binary_dialated.cols; j++) {
            image.at<cv::Vec3b>(i, j)[0] = (255 - binary_dialated.at<uchar>(i, j)) & image.at<cv::Vec3b>(i, j)[0];
            image.at<cv::Vec3b>(i, j)[1] = (255 - binary_dialated.at<uchar>(i, j)) & image.at<cv::Vec3b>(i, j)[1];
            image.at<cv::Vec3b>(i, j)[2] = (255 - binary_dialated.at<uchar>(i, j)) & image.at<cv::Vec3b>(i, j)[2];
        }
    }
    return image;
}

void ReadParameterFromFile() {
    static bool read_parameters = false;
    if (!read_parameters) {
        std::cout << "Reading Obstacle Removal Parameters from file" << std::endl;
    }
    return;
}
