#include "ncs_con/coordinates.h"

#define COORD_DEBUG

Coordinates::Coordinates():mcm(NULL), num_initialization_samples(20) {}

Coordinates::Coordinates(MultiCamMapper* const _mcm, const double _w, const double _d, const double _t, const double _h, const int _num_pts):mcm(_mcm), num_initialization_samples(20){
    init(_mcm, _w, _d, _t, _h, _num_pts);
}

void Coordinates::init(MultiCamMapper* const _mcm, const double _w, const double _d, const double _t, const double _h, const int _num_pts){
    w = _w; d = _d; t = _t; h = _h; num_pts = _num_pts;
    count_initialization = 0;
    markers_in_root = get_markers_pos_in_root(*mcm, num_pts); 
    markers_in_body = get_drone_marker_pts(w, d, t, h, num_pts);
    rM_T_B = estimate_transform_from_drone_to_root_marker(markers_in_body, markers_in_root);
}

/* Generate the point set of the position of marker centers in body frame.
 * @Param
 *      - w: width of the drone
 *      - d: distance of markers in the same plane
 *      - t: thickness of markers
 * @Return
 *      - pts: vector of points in body frame
 */
cv::Mat Coordinates::get_drone_marker_pts(const double w, const double d, const double t, const double h, const int num_pts) {
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
cv::Mat Coordinates::get_markers_pos_in_root(MultiCamMapper& mcm, const int num_pts) {
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
vector<cv::Point3f> Coordinates::mat_to_vec_points(cv::Mat& m){
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
cv::Mat Coordinates::estimate_transform_from_drone_to_root_marker(cv::Mat& realMarkerPos, cv::Mat& rootMarkerPos) {
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

    cv::Mat rM_T_B = cv::Mat::eye(cv::Size(4, 4), transform.type());
    transform.copyTo(rM_T_B(cv::Range(0, 3), cv::Range(0, 4)));
    cout << "Transformation from root marker to body frame:" << endl << fmt->format(rM_T_B) << endl;
    return rM_T_B;
}

void Coordinates::render(vector<cv::Mat>& imgs, float axis_size){
    MultiCamMapper::MatArrays mat_arrays = mcm->get_mat_arrays();
    // No pose is estimated
    if(mat_arrays.object_to_global.m.count(0) == 0)
        return;
        
    for(size_t j=0; j<imgs.size(); j++){
        draw_body_frame(imgs[j], 0, j, mat_arrays, axis_size);
        draw_inertial_frame(imgs[j], 0, j, mat_arrays, axis_size * 2);
    }
}

/* Draw the axes of the body frame in given camera's image
 * @Param
 *      - img: The image to render.
 *      - frame_id: Set to 0 for realtime tracking.
 *      - camera_index: The index of camera.
 *      - mat_arrays: The mat_arrays calculated by MultiCamMapper.
 *      - axis_size: The size of axis to render, unit in meters.
 */
void Coordinates::draw_body_frame(cv::Mat& img, int frame_id, int camera_index, MultiCamMapper::MatArrays& mat_arrays, float axis_size){
    // Transformation from body to root camera
    cv::Mat transform = mat_arrays.transforms_to_local_cam.v[camera_index] * 
        mat_arrays.object_to_global[frame_id] * 
        rM_T_B;
    
    draw_coordinate(img, camera_index, mat_arrays, transform, axis_size);
}

void Coordinates::draw_inertial_frame(cv::Mat& img, int frame_id, int camera_index, MultiCamMapper::MatArrays& mat_arrays, float axis_size){
    if (!is_initialized()) return;

    cv::Mat transform = mat_arrays.transforms_to_local_cam.v[camera_index] * rC_T_I;
    draw_coordinate(img, camera_index, mat_arrays, transform, axis_size);
}

void Coordinates::draw_coordinate(cv::Mat& img, int camera_index, MultiCamMapper::MatArrays& mat_arrays, const cv::Mat transform, float axis_size){
    int camera_id = mat_arrays.cam_mats.id[camera_index];
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

bool Coordinates::calc_pos_in_global(cv::Mat& out){
    static cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
    static cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);

    MultiCamMapper::MatArrays mat_arrays = mcm->get_mat_arrays();
    cv::Mat rC_T_B = mat_arrays.object_to_global[0] * rM_T_B;

    if (!is_initialized()){
        // Not initialized
        cv::Mat nrvec;
        cv::Rodrigues(rC_T_B(cv::Range(0, 3), cv::Range(0, 3)), nrvec);
        rvec += nrvec;
        tvec += rC_T_B(cv::Range(0, 3), cv::Range(3, 4));

        if (count_initialization == num_initialization_samples - 1){
            rvec /= (double)num_initialization_samples;
            tvec /= (double)num_initialization_samples;
            rC_T_I = vec_to_transform(rvec, tvec);
            I_T_rC = transform_inverse(rC_T_I);
            ROS_INFO("Global frame initialized");
        }

        count_initialization++;
        return false;
    }
    else{
        // Initialized
        I_T_B = I_T_rC * rC_T_B;
        I_T_B.copyTo(out);
        return true;
    }
}

cv::Mat Coordinates::vec_to_transform(cv::Mat& rvec, cv::Mat& tvec){
    cv::Mat trans = cv::Mat::eye(4, 4, rvec.type());
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    R.copyTo(trans(cv::Range(0, 3), cv::Range(0, 3)));
    tvec.copyTo(trans(cv::Range(0, 3), cv::Range(3, 4)));
    return trans;
}

cv::Mat Coordinates::transform_inverse(cv::Mat& trans){
    cv::Mat R = trans(cv::Range(0, 3), cv::Range(0, 3));
    cv::Mat t = trans(cv::Range(0, 3), cv::Range(3, 4));
    cv::Mat R_inv = R.t();
    cv::Mat inv = cv::Mat::eye(4, 4, trans.type());
    R_inv.copyTo(inv(cv::Range(0, 3), cv::Range(0, 3)));
    cv::Mat t_inv = -R_inv * t;
    t_inv.copyTo(inv(cv::Range(0, 3), cv::Range(3, 4)));
    return inv;
}
