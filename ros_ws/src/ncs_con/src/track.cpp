#include "ros/ros.h"

#include "pose_est/dataset.h"
#include "pose_est/multicam_mapper.h"
#include "pose_est/initializer.h"
#include <chrono>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <ncs_con/Con_msg_2d.h>
#include <ncs_con/cam_msg_2d.h>
#include <ncs_con/obj_msg_2d.h>

#include <opencv2/core/hal/interface.h>

using namespace std;
size_t num_cams = 3;

Initializer initializer;
MultiCamMapper mcm;

void print_usage(){
    cout<<"Usage: <path_to_solution_file>"<<endl;
}

cv::Mat make_mosaic(vector<cv::Mat> images, int mosaic_width){
    cv::Mat mosaic;
    if(images.size()==0)
        return mosaic;

    double image_h_over_w = double(images[0].rows)/images[0].cols;

    int num_images=images.size();

    int side_images=ceil(sqrt(num_images));

    int image_width=mosaic_width/side_images;
    int image_height=image_width*image_h_over_w;

    int width_images=side_images;
    int height_images=ceil(double(num_images)/side_images);

    mosaic=cv::Mat::zeros(image_height*height_images,image_width*width_images,images[0].type());
    for(int i=0;i<num_images;i++){
        int r = i/width_images;
        int c = i%width_images;
        cv::Mat img_resized;
        cv::resize(images[i],img_resized,cv::Size(image_width,image_height));
        img_resized.copyTo(mosaic(cv::Range(r*image_height,(r+1)*image_height),cv::Range(c*image_width,(c+1)*image_width)));
    }

    cv::line(mosaic, cv::Point(0, image_height), cv::Point(mosaic_width, image_height), cv::Scalar(255, 255, 255));
    cv::line(mosaic, cv::Point(image_width, 0), cv::Point(image_width, image_height*2), cv::Scalar(255, 255, 255));
    return mosaic;
}

void msgCallback(const ncs_con::Con_msg_2d::ConstPtr& msg) {
    const int min_detections_per_marker = 1;
    auto start=chrono::system_clock::now();

    ROS_INFO_STREAM(*msg);
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
            if (markers_count[obj.oid] > min_detections_per_marker){
                // Constructing corners
                vector<cv::Point2f> corners;
                for (int i=0; i<4; i++)
                    corners.push_back(cv::Point2f(obj.pos[2*i], obj.pos[2*i+1]));
                
                // Constructing marker
                aruco::Marker m = aruco::Marker(corners, obj.oid);

                cam_marker.push_back(m);
            }
        }

        if (cam_marker.size() > 0)
            valid_markers[cam.cid] = cam_marker;
    }

    vector<vector<vector<aruco::Marker>>> frame_detections(1);
    frame_detections[0] = valid_markers;

    int c = 0;
    for (auto cam: valid_markers){
        std::cout << "cam" << c << ": ";
        const size_t s = 10;
        int place[s] = {0};
        for (auto marker: cam)
            place[marker.id] = 1;
        for (int m=0; m<s; m++){
            std::cout << ' ' << place[m];
        }
        std::cout << std::endl;
        c++;
    }
    c = 0;
    for (auto& cam: valid_markers){
        std::cout << "cam" << c << ": " << std::endl;
        for (auto& marker : cam){
            std::cout << "marker" << marker.id << ": " << marker << " from cam " << c << std::endl;
        }
        c++;
    }

    size_t num_wi_frames=0;
    double sum_detections_duration=0;
    double sum_inf_duration=0;
    double sum_duration=0;
    //initiate the parameters
    initializer.set_detections(frame_detections);

    int num_detections=0;
    for(size_t i=0;i<frame_detections[0].size();i++)
        num_detections+=frame_detections[0][i].size();

    if(num_detections>0){

        num_wi_frames++;

        initializer.obtain_pose_estimations();
        initializer.init_object_transforms();

        std::cout << "obj_trans: ";
        for (auto& o: initializer.get_object_transforms()){
            std::cout << o.first << ", ";
        }
        std::cout << std::endl;

        std::cout << "obj_trans: ";
        for (auto& o: initializer.get_frame_cam_markers()){
            std::cout << o.first << ", ";
        }
        std::cout << std::endl;

        mcm.init(initializer.get_object_transforms(),initializer.get_frame_cam_markers());
        mcm.track();
    }
    chrono::duration<double> duration=chrono::system_clock::now()-start;

    // Visualizing
    vector<cv::Mat> frames;
    for(size_t i=0; i<3; i++){
        frames.push_back(cv::Mat::zeros(480, 640, 16));
    }

    for (auto& cam: msg->cams){
        for (auto& obj: cam.objs){
            int cid = obj.cid;
            int oid = obj.oid;
            if (cid >= 3 || cid < 0){
                std::cout << "invalid cid " << cid << std::endl;
                continue;
            }
            cv::Scalar color(255, 255, 0);
            if (markers_count.find(obj.oid) == markers_count.end()){
                // invalid marker
                color = cv::Scalar(255, 0, 0);
            }
            vector<vector<cv::Point>> corners(1);
            for (int i=0; i<4; i++)
                corners[0].push_back(cv::Point(obj.pos[2*i], obj.pos[2*i+1]));
            const cv::Point* pts = &corners[0][0];
            int n = (int)corners[0].size();

            cv::polylines(frames[cid], &pts, &n, 1, true, color);
            cv::putText(frames[cid], std::to_string(cid) + ":" + std::to_string(oid), cv::Point(obj.pos[0], obj.pos[1]),
                cv::FONT_HERSHEY_SIMPLEX, 1, color,1);
        }
    }
    
    for(size_t j=0; j<frame_detections[0].size(); j++){
        if(num_detections==0)
            cv::putText(frames[j],"no reliable detections",cv::Point(100,100),cv::FONT_HERSHEY_SIMPLEX,1.5,cv::Scalar(0,0,255),3);
        else
            mcm.overlay_markers(frames[j],0,j);
    }
    cv::Mat mosaic=make_mosaic(frames, 840);
    std::cout << "mat size " << mosaic.size() << std::endl;
    cv::imshow("Tracking",mosaic);
    cv::waitKey(1);
}

int main(int argc, char *argv[]){
    if(argc<2){
        print_usage();
        return -1;
    }
    string folder_path(argv[1]);

    vector<CamConfig> cam_configs;

    cam_configs=CamConfig::read_cam_configs(folder_path);

    std::cout << "config size " << cam_configs.size() << std::endl;
    int c = 0;
    for (auto config: cam_configs){
        std::cout << "config" << c << std::endl;
        std::cout << config.getCamMat() << std::endl;
        std::cout << config.getDistCoeffs() << std::endl;
        std::cout << config.getImageSize() << std::endl;
        c++;
    }

    string solution_file_path = folder_path + "/final.solution";
    mcm.read_solution_file(solution_file_path);

    map<int,cv::Mat> transforms_to_root_cam,transforms_to_root_marker;

    MultiCamMapper::MatArrays ma=mcm.get_mat_arrays();
    auto mcm_ttrc=ma.transforms_to_root_cam;
    auto mcm_ttrm=ma.transforms_to_root_marker;
    for(auto &cam_id_index: mcm_ttrc.m){
        int cam_id=cam_id_index.first;
        int cam_index=cam_id_index.second;
        transforms_to_root_cam[cam_id]=mcm_ttrc.v[cam_index];
    }
    for(auto &marker_id_index: mcm_ttrm.m){
        int marker_id=marker_id_index.first;
        int marker_index=marker_id_index.second;
        transforms_to_root_marker[marker_id]=mcm_ttrm.v[marker_index];
    }

    initializer = Initializer(mcm.get_marker_size(),cam_configs);
    initializer.set_transforms_to_root_cam(transforms_to_root_cam);
    initializer.set_transforms_to_root_marker(transforms_to_root_marker);

    mcm.set_optmize_flag_cam_poses(false);
    mcm.set_optmize_flag_marker_poses(false);
    mcm.set_optmize_flag_object_poses(true);

    ROS_INFO("Start tracking");

    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("ncscon_topic", 1000, msgCallback);

    ros::spin();

    ROS_INFO("Stop tracking");
    
    return 0;
}
