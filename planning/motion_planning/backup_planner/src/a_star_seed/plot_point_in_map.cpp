//
//  a_star_seed.cpp
//  AStarSeed
//
//  Created by Satya Prakash on 06/09/13.
//  Copyright (c) 2013 Satya Prakash. All rights reserved.
//


#include "a_star_seed/a_star_seed.hpp"
namespace navigation {
    void AStarSeed::plotPointInMap(const State & pos)
    {
        cv::circle(image, cv::Point(pos.x(), image.rows- pos.y() - 1), 3, cv::Scalar(255), -1);
        
    }
}
