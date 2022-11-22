#ifndef TRACKER_H
#define TRACKER_H

#include "ros/ros.h"

#include "pose_est/dataset.h"
#include "pose_est/multicam_mapper.h"
#include "pose_est/initializer.h"
#include "ncs_con/coordinates.h"
#include <chrono>

// Opencv packages
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/hal/interface.h>

// ROS Messages
#include <ncs_con/Con_msg_2d.h>
#include <ncs_con/cam_msg_2d.h>
#include <ncs_con/obj_msg_2d.h>
#include <ncs_con/Est_pose.h>

// #define TRACK_DEBUG
using namespace std;

class Tracker{
public:
    Tracker(string config_folder_path);

private:
    size_t num_cams = 3;
    ncs_con::Est_pose est_pose;
    
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pub;

    /* Modules for pose estimation */
    Initializer initializer;
    MultiCamMapper mcm;
    Coordinates coordinate;

    void msgCallback(const ncs_con::Con_msg_2d::ConstPtr& msg);

    /* Publish pose matrix */
    void publish_pose(cv::Mat pose);

    cv::Mat make_mosaic(vector<cv::Mat> images, int mosaic_width);
};

#endif //TRACKER_H