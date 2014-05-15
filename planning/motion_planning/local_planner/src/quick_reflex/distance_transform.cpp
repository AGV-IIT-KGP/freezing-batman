//
//  a_star_seed.cpp
//  AStarSeed
//
//  Created by Satya Prakash on 06/09/13.
//  Copyright (c) 2013 Satya Prakash. All rights reserved.
//

#include "quick_reflex/quick_reflex.hpp"

namespace navigation {

    void quickReflex::distanceTransform() {

        cv::Mat binaryImg, transformedImg;
        int i,j;
        threshold(fusionMap, binaryImg, 100, 255, CV_THRESH_BINARY);
        // int type= fusionMap.type();
        // cvtColor(fusionMap, binaryImg, CV_BGR2GRAY);
        
        binaryImg = 255 - binaryImg;
        cv::distanceTransform(binaryImg,transformedImg,CV_DIST_L2,3);
        float DtThresh = 100;
        for(i=0;i<fusionMap.rows;i++)
            for(j=0;j<fusionMap.cols;j++)
                if(transformedImg.at<float>(i,j)>DtThresh)
                    transformedImg.at<float>(i,j)=DtThresh;
        cv::normalize(transformedImg,transformedImg,0,1,cv::NORM_MINMAX);
        double minVal, maxVal;
        minMaxLoc(transformedImg,&minVal,&maxVal);
        binaryImg.convertTo(binaryImg,CV_8U, 255.0/(maxVal-minVal), -minVal*255.0/(maxVal- minVal));
        transformedImg.convertTo(binaryImg,CV_8U, 255.0/(maxVal-minVal), -minVal*255.0/(maxVal- minVal));
        // cv::threshold(transformedImg, transformedImg, .5, 1., CV_THRESH_BINARY);
        // transformedImg.convertTo(binaryImg, CV_8U);
        binaryImg = 255 - binaryImg;
        // type=binaryImg.type();
        fusionMap=binaryImg;
        // type=fusionMap.type();
   }
}
