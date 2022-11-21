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
#include <opencv2/calib3d.hpp>

using namespace std;
size_t num_cams = 3;

Initializer initializer;
MultiCamMapper mcm;
cv::Mat body_trans_to_root;

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

/* Generate the point set of the position of marker centers in body frame.
 * @Param
 *      - w: width of the drone
 *      - d: distance of markers in the same plane
 *      - t: thickness of markers
 * @Return
 *      - pts: vector of points in body frame
 */
cv::Mat get_drone_marker_pts(const double w, const double d, const double t, const double h, const int num_pts=8) {
    // vector<cv::Point3f> pts(num_pts), out_pts;
    // pts[0] = cv::Point3f(   d/2,  w/2+t, h);
    // pts[1] = cv::Point3f(  -d/2,  w/2+t, h);
    // pts[2] = cv::Point3f(-w/2-t,    d/2, h);
    // pts[3] = cv::Point3f(-w/2-t,   -d/2, h);
    // pts[4] = cv::Point3f(  -d/2, -w/2-t, h);
    // pts[5] = cv::Point3f(   d/2, -w/2-t, h);
    // pts[6] = cv::Point3f( w/2+t,   -d/2, h);
    // pts[7] = cv::Point3f( w/2+t,    d/2, h);
    cv::Mat pts = cv::Mat(3, num_pts, CV_64F);
    cv::Mat(cv::Point3f(   d/2,  w/2+t, h)).copyTo(pts.col(0));
    cv::Mat(cv::Point3f(  -d/2,  w/2+t, h)).copyTo(pts.col(1));
    cv::Mat(cv::Point3f(-w/2-t,    d/2, h)).copyTo(pts.col(2));
    cv::Mat(cv::Point3f(-w/2-t,   -d/2, h)).copyTo(pts.col(3));
    cv::Mat(cv::Point3f(  -d/2, -w/2-t, h)).copyTo(pts.col(4));
    cv::Mat(cv::Point3f(   d/2, -w/2-t, h)).copyTo(pts.col(5));
    cv::Mat(cv::Point3f( w/2+t,   -d/2, h)).copyTo(pts.col(6));
    cv::Mat(cv::Point3f( w/2+t,    d/2, h)).copyTo(pts.col(7));

    cv::Mat R2z = cv::getRotationMatrix2D(cv::Point2f(0, 0), 45.0f, 1.0f);
    cv::Mat Rz = cv::Mat::eye(3, 3, CV_64F);

    R2z.copyTo(Rz(cv::Rect_<int>(0,0,3,2)));
    return Rz * pts;
}

/* Extract the marker's positions expressed in root marker frame.
 * @Param
 *      - mcm: MultiCamMapper
 *      - num_pts: Number of markers
 * @Return
 *      - pts: vector of points in root marker frame
 */
cv::Mat get_markers_pos_in_root(MultiCamMapper& mcm, const int num_pts=8) {
    cv::Mat pts = cv::Mat(3, num_pts, CV_64F);

    int marker_list[num_pts] = {0};
    MultiCamMapper::MatArrays mat_arrays = mcm.get_mat_arrays();
    cout << "Pair of markers: " << endl;
    for(int marker_index=0; marker_index<mat_arrays.transforms_to_root_marker.id.size(); marker_index++){
        int marker_id = mat_arrays.transforms_to_root_marker.id[marker_index];

        if (marker_id >= 1 && marker_id <= 8){
            marker_list[marker_id-1] += 1;
            cv::Mat pt = mat_arrays.transforms_to_root_marker.v[marker_index](cv::Range(0,3),cv::Range(3,4));
            pt.copyTo(pts.col(marker_id-1));
            cout << "id: " << marker_id << ", index: " << marker_index << pt << endl;
        }
    }

    // check for markers
    int valid_counter = 0;
    cout << "Marks for drone estimation:" << endl;
    for (int i=0; i<num_pts; i++){
        cout << marker_list[i] << ", ";
        if (marker_list[i] > 0)
            valid_counter++;
    }
    cout << endl;

    if (valid_counter == num_pts)
        cout << "Number of detected markers is correct!" << endl;
    else
        cout << "Error: Do not have enough markers for drone coordinate estimation. (Some of the markers are not detected)" << endl;
    return pts;
}

/* Transform matrix into vector of points
 * @Param
 *      - m: matrix to transform, with shape 3 x n.
 * @Return
 *      - pts: vector of points
 */
vector<cv::Point3f> mat_to_vec_points(cv::Mat& m){
    vector<cv::Point3f> pts(m.cols);
    for (int c=0; c<m.cols; c++)
        pts[c] = cv::Point3f(m.col(c));
    return pts;
}

/* Estimate the affine transform from body frame to root marker frame
 * @Param
 *      - realMarkerPos: The markers expressed in body frame
 *      - roolMarkerPos: The markers expressed in root marker frame
 * @Return
 *      - r_T_b: A 4x4 transformation matrix
 */
cv::Mat estimate_transform_from_drone_to_root_marker(cv::Mat& realMarkerPos, cv::Mat& rootMarkerPos) {
    cv::Mat transform;
    vector<uchar> inliers;

    cv::Ptr<cv::Formatter> fmt = cv::Formatter::get(cv::Formatter::FMT_DEFAULT);
    fmt->set64fPrecision(4);
    fmt->set32fPrecision(4);

    cout << "root marker" << endl;
    cout << fmt->format(rootMarkerPos) << endl;
    cout << "real marker" << endl;
    cout << fmt->format(realMarkerPos) << endl;
    auto root_pts = mat_to_vec_points(rootMarkerPos);
    auto real_pts = mat_to_vec_points(realMarkerPos);
    transform = cv::estimateAffine3D(real_pts, root_pts);

    /* Checking for transforming results */
    cv::Mat out = transform(cv::Rect_<int>(0,0,3,3)) * realMarkerPos;
    for (int i=0; i<out.cols; i++)
        out.col(i) += transform(cv::Range(0,3),cv::Range(3,4));
    cout << "Transforming results " << endl << fmt->format(out) << endl;

    cv::Mat r_T_b = cv::Mat::eye(cv::Size(4, 4), transform.type());
    transform.copyTo(r_T_b(cv::Range(0, 3), cv::Range(0, 4)));
    cout << "Transformation from root marker to body frame:" << endl << fmt->format(r_T_b) << endl;
    return r_T_b;
}

/* Draw the axes of the body frame in given camera's image
 * @Param
 *      - img: The image to render.
 *      - frame_id: Set to 0 for realtime tracking.
 *      - camera_index: The index of camera.
 *      - mat_arrays: The mat_arrays calculated by MultiCamMapper.
 *      - axis_size: The size of axis to render, unit in meters.
 */
void draw_body_frame(cv::Mat& img, int frame_id, int camera_index, MultiCamMapper::MatArrays& mat_arrays, float axis_size){
    int camera_id = mat_arrays.cam_mats.id[camera_index];

    // No pose is estimated
    if(mat_arrays.object_to_global.m.count(frame_id) == 0)
        return;

    // Transformation from body to root camera
    cv::Mat transform = mat_arrays.transforms_to_local_cam.v[camera_index] * 
        mat_arrays.object_to_global[frame_id] * 
        body_trans_to_root;
    cv::Mat rvec; 
    cv::Rodrigues(transform(cv::Range(0,3),cv::Range(0,3)), rvec);
    cv::Mat tvec = transform(cv::Range(0,3), cv::Range(3,4));

    cv::Mat objectPoints(3, 4, CV_32FC1);
    objectPoints.at<float>(0, 0) = 0;
    objectPoints.at<float>(1, 0) = 0;
    objectPoints.at<float>(2, 0) = 0;
    objectPoints.at<float>(0, 1) = axis_size;
    objectPoints.at<float>(1, 1) = 0;
    objectPoints.at<float>(2, 1) = 0;
    objectPoints.at<float>(0, 2) = 0;
    objectPoints.at<float>(1, 2) = axis_size;
    objectPoints.at<float>(2, 2) = 0;
    objectPoints.at<float>(0, 3) = 0;
    objectPoints.at<float>(1, 3) = 0;
    objectPoints.at<float>(2, 3) = axis_size;

    vector<cv::Point2f> points_2d;
    cv::projectPoints(objectPoints, rvec, tvec, mat_arrays.cam_mats[camera_id], mat_arrays.dist_coeffs[camera_id], points_2d);
    // draw lines of different colours
    cv::line(img, points_2d[0], points_2d[1], cv::Scalar(0, 0, 255, 255), 1, cv::LINE_AA);
    cv::line(img, points_2d[0], points_2d[2], cv::Scalar(0, 255, 0, 255), 1, cv::LINE_AA);
    cv::line(img, points_2d[0], points_2d[3], cv::Scalar(255, 0, 0, 255), 1, cv::LINE_AA);
    cv::putText(img, "x", points_2d[1], cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255, 255), 2);
    cv::putText(img, "y", points_2d[2], cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0, 255), 2);
    cv::putText(img, "z", points_2d[3], cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0, 255), 2);
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
        frames.push_back(cv::Mat::zeros(480, 640, CV_8UC3));
    }

    for (auto& cam: msg->cams){
        for (auto& obj: cam.objs){
            int cid = obj.cid;
            int oid = obj.oid;
            if (cid >= 3 || cid < 0){
                std::cout << "invalid cid " << cid << std::endl;
                continue;
            }
            cv::Scalar color(255, 128, 0);
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
    
    MultiCamMapper::MatArrays mat_arrays = mcm.get_mat_arrays();
    for(size_t j=0; j<frame_detections[0].size(); j++){
        if(num_detections==0)
            cv::putText(frames[j],"no reliable detections",cv::Point(100,100),cv::FONT_HERSHEY_SIMPLEX,1.5,cv::Scalar(0,0,255),3);
        else{
            mcm.overlay_markers(frames[j],0,j);
            draw_body_frame(frames[j], 0, j, mat_arrays, 0.1f);
        }
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

    cv::Mat realMarkerPose = get_drone_marker_pts(0.300f, 0.230f, 0.002f, 0.001f);
    cv::Mat rootMarkerPos = get_markers_pos_in_root(mcm, 8);
    body_trans_to_root = estimate_transform_from_drone_to_root_marker(realMarkerPose, rootMarkerPos);

    ROS_INFO("Start tracking");

    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("ncscon_topic", 1000, msgCallback);

    ros::spin();

    ROS_INFO("Stop tracking");
    
    return 0;
}
