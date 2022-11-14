#include "ros/ros.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <set>
#include "pose_est/aruco_serdes.h"
// #include "aruco/aruco.h"
#include "pose_est/dataset.h"
#include "pose_est/multicam_mapper.h"

#include "std_msgs/String.h"

enum PUB_MODE {M2d, M3d};
const PUB_MODE pub_mode = PUB_MODE::M2d;
const int min_detections_per_marker = 1;

#include <ncs_con/Con_msg_2d.h>
#include <ncs_con/cam_msg_2d.h>
#include <ncs_con/obj_msg_2d.h>
// if (pub_mode == PUB_MODE::M2d){
// }

using namespace std;

ofstream output_file;
size_t num_cams = 3;

int print_usage(char* argv0) {
    cout << "Usage: " << argv0 << " <path_to_data_folder> [-d <dictionary>]"
         << endl
         << "The default values:" << endl
         << "\t<dictionary>: ARUCO_MIP_36h12" << endl;
    return -1;
}

int get_cam_id(int cid){
    return cid - 1;
}

void msgCallback(const ncs_con::Con_msg_2d::ConstPtr& msg) {
    if (!output_file.is_open()){
        ROS_WARN("CALLBACK with output file not ready!");
        return;
    }

    // marker count
    map<int, int> markers_count;
    for (auto& cam: msg->cams){
        for (auto& obj: cam.objs){
            if (markers_count.find(obj.oid) != markers_count.end())
                markers_count[obj.oid]++;
            else
                markers_count[obj.oid] = 1;
        }
    }

    // Logging markers
    for (auto it=markers_count.begin(); it!=markers_count.end(); ++it){
        std::cout << "Marker " << it->first << ": " << it->second << std::endl;
    }

    vector<vector<aruco::Marker>> valid_markers(num_cams);
    for (auto& cam: msg->cams){
        vector<aruco::Marker> cam_marker;
        for (auto& obj: cam.objs){
            if (markers_count[obj.oid] >= min_detections_per_marker){
                // Constructing corners
                vector<cv::Point2f> corners;
                for (int i=0; i<4; i++)
                    corners.push_back(cv::Point2f(obj.pos[2*i], obj.pos[2*i+1]));
                
                // Constructing marker
                aruco::Marker m = aruco::Marker(corners, obj.oid);

                cam_marker.push_back(m);
            }
        }
        int cid = cam.cid;
        valid_markers[get_cam_id(cid)] = cam_marker;
    }

    for (auto& cam: valid_markers){
        // Writing to file
        auto valid_markers_size = cam.size();
        output_file.write((char*)&valid_markers_size,
                            sizeof(valid_markers_size));
        for (auto& m: cam) {
            ArucoSerdes::serialize_marker(m, output_file);
        }
    }

    // ROS_INFO_STREAM(msg);
}

int main(int argc, char* argv[]) {
    if (argc < 2) return print_usage(argv[0]);

    string dictionary_name = "ARUCO_MIP_36h12";

    if (argc > 2) {
        vector<string> optional_params;
        for (int i = 3; i < argc; i++) optional_params.push_back(argv[i]);

        auto it = find(optional_params.begin(), optional_params.end(), "-d");
        if (it != optional_params.end()) dictionary_name = *(++it);
    }

    string folder_path = argv[1];
    string output_file_name = folder_path + "/aruco.detections";

    output_file = ofstream(output_file_name, ios_base::binary);
    if (!output_file.is_open())
        throw runtime_error("Could not open a file to write output at: " +
                            output_file_name);

    output_file.write((char*)&num_cams, sizeof(num_cams));

    ROS_INFO("Start recording");

    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("ncscon_topic", 1000, msgCallback);

    ros::spin();

    ROS_INFO("Stop recording");
    output_file.close();
    return 0;
}
