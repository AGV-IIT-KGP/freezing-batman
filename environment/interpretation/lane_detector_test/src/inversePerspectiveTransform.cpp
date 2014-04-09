#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <cassert>
#include <iostream>
#include <stdio.h>
#include "laneDetector.hpp"

cv::Point2f src_vertices[4];

int no_clicks = 0;

const int bot_x = 500;
const int bot_y = 900;

const int d = 100;
const int w = 100;
const int h = 100;

cv::Mat TransformImage(cv::Mat &image){

    cv::Point2f dst_vertices[4];

    dst_vertices[0] = cv::Point(bot_x - w/2, 900 - d - h);
    dst_vertices[1] = cv::Point(bot_x + w/2, 900 - d - h);
    dst_vertices[2] = cv::Point(bot_x - w/2, 900 - d );
    dst_vertices[3] = cv::Point(bot_x + w/2, 900 - d );

    cv::Mat wrap_perspective_transform = cv::getPerspectiveTransform(src_vertices, dst_vertices);

    cv::Mat result;

    cv::warpPerspective(image, result, wrap_perspective_transform, cv::Size(1000, 1000), cv::INTER_NEAREST , cv::BORDER_CONSTANT);

    return result;
}

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
     bool *done = (bool*)userdata;
     if  ( event == cv::EVENT_LBUTTONDOWN ){

         if(no_clicks < 4){
             src_vertices[no_clicks] = cv::Point(x, y);
             std::cout << "x: " << x << "y: " << y << std::endl;
             no_clicks++;
             if(no_clicks == 4) *done = true;
         }
         else{
             *done = true;
         }
     }
}

cv::Mat LaneDetector::InversePerspectiveTransform(cv::Mat &image){
    static bool done = false;
    static bool read_parameters = false;

    if (debug_mode == 5 && !done){
        cv::namedWindow("Original Image");
        cv::setMouseCallback("Original Image", CallBackFunc, &done);

        while(!done){
            cv::imshow("Original Image",image);
            cv::waitKey(10);
        }
        cv::destroyWindow("Original Image");


        // Write the parameters to file
        FILE* ipt_data = fopen("data/ipt.dat", "w");

        for(int i=0; i < 4; i++){

            fprintf(ipt_data, "%f %f\n", src_vertices[i].x, src_vertices[i].y);
        }

        fclose(ipt_data);

        cv::Mat result = TransformImage(image);
        debug_mode = 6;
        return result;
    }

    else{
        if(!read_parameters){
            std::cout << "Reading Inverse Perspective Transform parameters from file" << std::endl;
            FILE* ipt_data = fopen("data/ipt.dat", "r");

            for(int i=0; i < 4; i++){
                fscanf(ipt_data, "%f %f", &src_vertices[i].x, &src_vertices[i].y);
                std::cout << " x: " << src_vertices[i].x << " y: " << src_vertices[i].y << std::endl;
            }

            fclose(ipt_data);
            read_parameters = true;
        }

        cv::Mat result = TransformImage(image);
        return result;

    }
}
