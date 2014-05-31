#include <laneDetector.hpp>

cv::Mat applyCanny(cv::Mat &img, int debug) {
    cv::Mat grayscale_image(img.rows, img.cols, CV_8UC1, cvScalarAll(0));
    cv::Mat canny_image(img.rows, img.cols, CV_8UC1, cvScalarAll(0));
    cv::cvtColor(img, grayscale_image, CV_BGR2GRAY);
    int minthreshold = 180, maxthreshold = 255;
    if (debug == 3) {
        cv::namedWindow("canny_control_box", 1);
        cv::createTrackbar("minthreshold", "canny_control_box", &minthreshold, 255);
        cv::createTrackbar("maxthreshold", "canny_control_box", &maxthreshold, 255);
    }
    do {
        cv::Canny(grayscale_image, canny_image, minthreshold, maxthreshold, 3);
        if (debug == 3) {
            cv::imshow("canny_control_box", canny_image);
        }
        if (cv::waitKey(33) == 27)
            break;

    } while (debug == 3);
    return canny_image;
} // converts the given image into a binary one using canny

cv::Mat applyHough(cv::Mat &img, int debug) {
    cv::Mat hough_image(img.rows, img.cols, CV_8UC1, cvScalarAll(0));
    int minpoints = 5, maxpoints = 10;
    if (debug == 3) {
        cv::namedWindow("hough_control_box", 1);
    }
    cv::createTrackbar("minpoints", "hough_control_box", &minpoints, 255);
    cv::createTrackbar("maxpoints", "hough_control_box", &maxpoints, 255);
    do {
        std::vector<cv::Vec4i>lines;
        cv::HoughLinesP(img, lines, 1, CV_PI / 180, maxpoints, minpoints, 60);
        for (size_t i = 0; i < lines.size(); i++) {
            cv::Vec4i l = lines[i];
            cv::line(hough_image, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255, 255, 255), 3, CV_AA);
        }
        if (debug == 3) {
            cv::imshow("hough_control_box", hough_image);
        }
        if (cv::waitKey(33) == 27) {
            break;
        }
    } while (debug == 3);
    return hough_image;
} // applying probablistic hough transformation on the canny image obtained

cv::Mat applyThreshold(cv::Mat &img, int debug) {
    int bin_threshold = 180;

    cv::Mat grayscale_image(img.rows, img.cols, CV_8UC1, cvScalarAll(0));
    cv::Mat threshold_image(img.rows, img.cols, CV_8UC1, cvScalarAll(0));

    if (debug == 4) {
        cv::namedWindow("threshold_control_box", 1);
        cv::createTrackbar("bin_threshold", "threshold_control_box", &bin_threshold, 255);
    }

    cv::inRange(img, cv::Scalar(bin_threshold, bin_threshold, bin_threshold), cv::Scalar(256, 256, 256), threshold_image);

    return threshold_image;
} //  converting a given image into binary using a threshold_image

cv::Mat mergeBinaryImages(cv::Mat &bin_img1, cv::Mat &bin_img2, int debug) {
    cv::Mat merged_image(bin_img1.rows, bin_img1.cols, CV_8UC1, cvScalarAll(0));
    do {
        for (int i = 0; i < bin_img1.rows; i++) {
            for (int j = 0; j < bin_img1.cols; j++) {
                if (bin_img1.at<uchar>(i, j) == 255 && bin_img2.at<uchar>(i, j) == 255) {
                    merged_image.at<uchar>(i, j) = 255;
                }
            }
        }
        if (debug) {
            cv::namedWindow("Binary Merged Image");
            cv::imshow("Binary Merged Image", merged_image);
        }
        if (cv::waitKey(33) == 27) {
            break;
        }
    } while (debug);
    return merged_image;
}// merging two binary images..

cv::Mat LaneDetector::getLaneBinary(cv::Mat &image) {
    cv::Mat threshold_image(image.rows, image.cols, CV_8UC1, cvScalarAll(0));
    cv::Mat hough_image(image.rows, image.cols, CV_8UC1, cvScalarAll(0));
    threshold_image = applyThreshold(image, debug_mode);
    hough_image = applyHough(threshold_image, debug_mode);
    return hough_image;
} // Detect lanes and return a binary image with Lanes only
