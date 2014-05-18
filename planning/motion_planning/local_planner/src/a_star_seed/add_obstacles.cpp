//
//  a_star_seed.cpp
//  AStarSeed
//
//  Created by Satya Prakash on 06/09/13.
//  Copyright (c) 2013 Satya Prakash. All rights reserved.
//

#include "a_star_seed/a_star_seed.hpp"

namespace navigation {

    void addObstacles(cv::Mat& map, const int num_of_obstacles_p = 0) {
        srand((unsigned int) time(NULL));
        cv::circle(map, cv::Point(500, map.rows - 300 - 1), 100, cv::Scalar(255), -1);

        int num_of_obstacles = num_of_obstacles_p;
        while (num_of_obstacles--) {
            const int x = rand() % 800;
            const int y = rand() % 800;
            const int radius = rand() % 40 + 20;

            const int x1 = rand() % 100;
            const int y1 = rand() % 100;
            cv::circle(map, cv::Point(x, map.rows - y - 1), radius, cv::Scalar(255), -1);
        }
    }
}
