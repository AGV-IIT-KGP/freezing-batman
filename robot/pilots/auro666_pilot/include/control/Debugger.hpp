/* 
 * File:   Debugger.hpp
 * Author: samuel
 *
 * Created on 7 January, 2014, 1:26 PM
 */

#ifndef DEBUGGER_HPP
#define	DEBUGGER_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Path.h>

class Debugger {
public:
    Debugger();
    Debugger(const Debugger& orig);
    virtual ~Debugger();

    void display(int debug_mode);
    void updateCurrentPath(const geometry_msgs::Pose::ConstPtr& _pose);
    void updateCTEPlotData(const std_msgs::Float64::ConstPtr& _cte);
    void updateTargetPath(const nav_msgs::Path::ConstPtr& path_ptr);

private:
    std::string cte_response;
    std::string path_tracking;
    
    std::vector<int> cte_plot;
    std::vector<cv::Point> flat_line;
    std::vector<cv::Point> traversed_path;
    std::vector<cv::Point> target_path;
    cv::Mat cte_image;
    cv::Mat path_image;
};

#endif	/* DEBUGGER_HPP */

