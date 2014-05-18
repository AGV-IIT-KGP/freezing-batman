//
//  a_star_seed.cpp
//  AStarSeed
//
//  Created by Satya Prakash on 06/09/13.
//  Copyright (c) 2013 Satya Prakash. All rights reserved.
//


#include "a_star_seed/a_star_seed.hpp"

namespace navigation {

    void AStarSeed::distanceTransform() {
        cv::Mat binaryImg, transformedImg;
        int i, j;
        

        
        threshold(fusionMap, binaryImg, 100, 255, CV_THRESH_BINARY);

        binaryImg = 255 - binaryImg;
        cv::distanceTransform(binaryImg, transformedImg, CV_DIST_L2, 3);
        float DtThresh = 100;
        for (i = 0; i < fusionMap.rows; i++)
            for (j = 0; j < fusionMap.cols; j++)
                if (transformedImg.at<float>(i, j) > DtThresh)
                    transformedImg.at<float>(i, j) = DtThresh;
        cv::normalize(transformedImg, transformedImg, 0, 1, cv::NORM_MINMAX);
        double minVal, maxVal;
        minMaxLoc(transformedImg, &minVal, &maxVal);
        binaryImg.convertTo(binaryImg, CV_8U, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));
        transformedImg.convertTo(binaryImg, CV_8U, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));

        binaryImg = 255 - binaryImg;
        fusionMap = binaryImg;
    }

    void quickReflex::distanceTransform() {
        cv::Mat binaryImg, transformedImg;
        int i, j;
        int BINARY_MIN_THRESHOLD, BINARY_MAX_THRESHOLD;
        nh.getParam("local_planner/max_threshold", BINARY_MAX_THRESHOLD);
        nh.getParam("local_planner/min_threshold", BINARY_MIN_THRESHOLD);
        threshold(fusionMap, binaryImg, 100, 255, CV_THRESH_BINARY);

        binaryImg = 255 - binaryImg;
        cv::distanceTransform(binaryImg, transformedImg, CV_DIST_L2, 3);
        float DtThresh = 100;
        for (i = 0; i < fusionMap.rows; i++)
            for (j = 0; j < fusionMap.cols; j++)
                if (transformedImg.at<float>(i, j) > DtThresh)
                    transformedImg.at<float>(i, j) = DtThresh;
        cv::normalize(transformedImg, transformedImg, 0, 1, cv::NORM_MINMAX);
        double minVal, maxVal;
        minMaxLoc(transformedImg, &minVal, &maxVal);
        binaryImg.convertTo(binaryImg, CV_8U, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));
        transformedImg.convertTo(binaryImg, CV_8U, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));

        binaryImg = 255 - binaryImg;
        fusionMap = binaryImg;
    }
}
