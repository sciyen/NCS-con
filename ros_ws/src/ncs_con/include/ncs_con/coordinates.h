#ifndef COORDINATES_H
#define COORDINATES_H

#include "pose_est/multicam_mapper.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

class Coordinates{
public:
    Coordinates();
    Coordinates(MultiCamMapper* const _mcm, const double _w, const double _d, const double _t, const double _h, const int _num_pts=8);
    
    void init(MultiCamMapper* const _mcm, const double _w, const double _d, const double _t, const double _h, const int _num_pts=8);

    void render(vector<cv::Mat>& img, float axis_size=0.1f);

private:
    double w, d, t, h;
    int num_pts;
    cv::Mat markers_in_root;
    cv::Mat markers_in_body;
    MultiCamMapper* mcm;

    cv::Mat r_T_b;

    static vector<cv::Point3f> mat_to_vec_points(cv::Mat& m);
    static cv::Mat estimate_transform_from_drone_to_root_marker(cv::Mat& realMarkerPos, cv::Mat& rootMarkerPos);
    static cv::Mat get_drone_marker_pts(const double w, const double d, const double t, const double h, const int num_pts=8);
    static cv::Mat get_markers_pos_in_root(MultiCamMapper& mcm, const int num_pts=8);
    static void draw_body_frame(cv::Mat& img, int frame_id, int camera_index, MultiCamMapper::MatArrays& mat_arrays, const cv::Mat trans_body_to_root, float axis_size);
};

#endif //COORDINATES_H
