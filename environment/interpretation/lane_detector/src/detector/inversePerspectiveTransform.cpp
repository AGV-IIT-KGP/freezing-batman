#include <cassert>
#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/package.h>
#include <laneDetector.hpp>

cv::Point2f src_vertices[4];

int no_clicks = 0;

int bot_x;
int bot_y;

int d;
int w;
int h;

std::string my_ipt_offsets_file;

void loadVariable() {
    int status = 1;
    FILE *readFile;
    readFile = fopen(my_ipt_offsets_file.c_str(), "r");
    status = status && fscanf(readFile, "bot_x = %d\n", &bot_x);
    status = status && fscanf(readFile, "bot_y = %d\n", &bot_y);
    status = status && fscanf(readFile, "d = %d\n", &d);
    status = status && fscanf(readFile, "w = %d\n", &w);
    status = status && fscanf(readFile, "h = %d\n", &h);
    printf("%d %d %d %d %d\n", bot_x, bot_y, d, w, h);
    if (status == 0) {
        printf("load in reading bot variable file\n");
    }
}

cv::Mat transformImage(cv::Mat &image) {
    static bool file_opened = false;

    if (!file_opened) {
        loadVariable();
        file_opened = true;
    }
    cv::Point2f dst_vertices[4];

    dst_vertices[0] = cv::Point(bot_x - w / 2, 900 - d - h);
    dst_vertices[1] = cv::Point(bot_x + w / 2, 900 - d - h);
    dst_vertices[2] = cv::Point(bot_x - w / 2, 900 - d);
    dst_vertices[3] = cv::Point(bot_x + w / 2, 900 - d);

    cv::Mat wrap_perspective_transform = cv::getPerspectiveTransform(src_vertices, dst_vertices);
    cv::Mat result;
    cv::warpPerspective(image, result, wrap_perspective_transform, cv::Size(1000, 1000), cv::INTER_NEAREST, cv::BORDER_CONSTANT);

    return result;
}

void callbackFunc(int event, int x, int y, int flags, void* userdata) {
    bool *done = (bool*)userdata;
    if (event == cv::EVENT_LBUTTONDOWN) {
        if (no_clicks < 4) {
            src_vertices[no_clicks] = cv::Point(x, y);
            std::cout << "x: " << x << "y: " << y << std::endl;
            no_clicks++;
            if (no_clicks == 4) *done = true;
        } else {
            *done = true;
        }
    }
}

cv::Mat LaneDetector::inversePerspectiveTransform(cv::Mat &image) {
    my_ipt_offsets_file =ros::package::getPath("lane_detector")+"/data/"+  ipt_offsets_file;

    static bool done = false;
    static bool read_parameters = false;

    if (debug_mode == 5 && !done) {
        cv::namedWindow("Original Image");
        cv::setMouseCallback("Original Image", callbackFunc, &done);

        while (!done) {
            cv::imshow("Original Image", image);
            cv::waitKey(10);
        }
        cv::destroyWindow("Original Image");

        // Write the parameters to file
        int status = 1;
        FILE* ipt_data = fopen(warp_matrix_file.c_str(), "w");

        for (int i = 0; i < 4; i++) {
            status = status && fprintf(ipt_data, "%f %f\n", src_vertices[i].x, src_vertices[i].y);
        }

        if (status == 0) {
            printf("Couldn't write to file ");
            printf(warp_matrix_file.c_str());
        }

        fclose(ipt_data);

        cv::Mat result = transformImage(image);
        debug_mode = 6;
        return result;
    } else {
        if (!read_parameters) {
            std::cout << "Reading Inverse Perspective Transform parameters from file" << std::endl;

            int status = 1;
            FILE* ipt_data = fopen((ros::package::getPath("lane_detector")+"/data/"+warp_matrix_file).c_str(), "r");

            for (int i = 0; i < 4; i++) {
                status = status && fscanf(ipt_data, "%f %f", &src_vertices[i].x, &src_vertices[i].y);
                std::cout << " x: " << src_vertices[i].x << " y: " << src_vertices[i].y << std::endl;
            }

            fclose(ipt_data);
            read_parameters = true;

            if (status == 0) {
                printf("Couldn't read file ");
                printf(warp_matrix_file.c_str());
            }
        }

        cv::Mat result = transformImage(image);
        return result;
    }
}
