//
//  a_star_seed.cpp
//  AStarSeed
//
//  Created by Satya Prakash on 06/09/13.
//  Copyright (c) 2013 Satya Prakash. All rights reserved.
//

#include <a_star_seed/a_star_seed.hpp>
#include <opencv2/imgproc/types_c.h>

namespace navigation {

    void AStarSeed::distanceTransform() {
        cv::Mat binary_img, transformed_img;
        int i, j;

        cv::threshold(fusion_map, binary_img, 100, 255, CV_THRESH_BINARY);

        binary_img = 255 - binary_img;
        cv::distanceTransform(binary_img, transformed_img, CV_DIST_L2, 3);
        //                float dt_threshold = 100;
        //                for (i = 0; i < fusion_map.rows; i++)
        //                    for (j = 0; j < fusion_map.cols; j++)
        //                        if (transformed_img.at<float>(i, j) > dt_threshold)
        //                            transformed_img.at<float>(i, j) = dt_threshold;

        cv::normalize(transformed_img, transformed_img, 0, 1, cv::NORM_MINMAX);
        double min_val, max_val;
        cv::minMaxLoc(transformed_img, &min_val, &max_val);
        binary_img.convertTo(binary_img, CV_8U, 255.0 / (max_val - min_val), -min_val * 255.0 / (max_val - min_val));
        transformed_img.convertTo(binary_img, CV_8U, 255.0 / (max_val - min_val), -min_val * 255.0 / (max_val - min_val));

        binary_img = 255 - binary_img;
        cv::rectangle(binary_img, cv::Point(0 * binary_img.cols, 0 * binary_img.rows), cv::Point(.1 * binary_img.cols, 1 * binary_img.rows), cv::Scalar(0, 0, 0), CV_FILLED, 8, 0);
        cv::rectangle(binary_img, cv::Point(.1 * binary_img.cols, 1 * binary_img.rows), cv::Point(1 * binary_img.cols, .895 * binary_img.rows), cv::Scalar(0, 0, 0), CV_FILLED, 8, 0);
        cv::rectangle(binary_img, cv::Point(.9 * binary_img.cols, .9 * binary_img.rows), cv::Point(1 * binary_img.cols, 0 * binary_img.rows), cv::Scalar(0, 0, 0), CV_FILLED, 8, 0);
        cv::rectangle(binary_img, cv::Point(.1 * binary_img.cols, 0 * binary_img.rows), cv::Point(.9 * binary_img.cols, .1 * binary_img.rows), cv::Scalar(0, 0, 0), CV_FILLED, 8, 0);
        fusion_map = binary_img;
    }

    void quickReflex::distanceTransform() {
        cv::Mat binary_img, transformed_img;
        int i, j;
        int binary_min_threshold, binary_max_threshold;
        float DtThresh = 200;
        node_handle.getParam("local_planner/max_threshold", binary_max_threshold);
        node_handle.getParam("local_planner/min_threshold", binary_min_threshold);
        cv::threshold(fusion_map, binary_img, 100, 255, CV_THRESH_BINARY);

        binary_img = 255 - binary_img;
        cv::distanceTransform(binary_img, transformed_img, CV_DIST_L2, 3);

       
        cv::normalize(transformed_img, transformed_img, 0, 1, cv::NORM_MINMAX);
        double minVal, maxVal;
        cv::minMaxLoc(transformed_img, &minVal, &maxVal);
        //binary_img.convertTo(binary_img, CV_8U, 400.0 / (maxVal - minVal), -minVal * 400.0 / (maxVal - minVal));
        transformed_img.convertTo(binary_img, CV_8U, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));

        binary_img = 255 - binary_img;
        cv::rectangle(binary_img, cv::Point(0 * binary_img.cols, 0 * binary_img.rows), cv::Point(.1 * binary_img.cols, 1 * binary_img.rows), cv::Scalar(0, 0, 0), CV_FILLED, 8, 0);
        cv::rectangle(binary_img, cv::Point(.1 * binary_img.cols, 1 * binary_img.rows), cv::Point(1 * binary_img.cols, .9 * binary_img.rows), cv::Scalar(0, 0, 0), CV_FILLED, 8, 0);
        cv::rectangle(binary_img, cv::Point(.9 * binary_img.cols, .9 * binary_img.rows), cv::Point(1 * binary_img.cols, 0 * binary_img.rows), cv::Scalar(0, 0, 0), CV_FILLED, 8, 0);
        cv::rectangle(binary_img, cv::Point(.1 * binary_img.cols, 0 * binary_img.rows), cv::Point(.9 * binary_img.cols, .1 * binary_img.rows), cv::Scalar(0, 0, 0), CV_FILLED, 8, 0);
        fusion_map = binary_img;
//         for (i = 0; i < fusion_map.rows; i++)
//            for (j = 0; j < fusion_map.cols; j++) {
//                if (fusion_map.at<uchar>(i, j) < DtThresh) {
//                    fusion_map.at<uchar>(i, j) = 0;
//                } else {
//                    fusion_map.at<uchar>(i, j) = (fusion_map.at<uchar>(i, j) - 205)*255 / 50;
//                }
//            }

        //cv::threshold(fusion_map, fusion_map, 245, 0, CV_THRESH_TOZERO);
    }
}
