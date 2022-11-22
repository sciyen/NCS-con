#include "ros/ros.h"

#include "ncs_con/tracker.h"

void print_usage(){
    cout<<"Usage: <path_to_solution_file>"<<endl;
}

int main(int argc, char *argv[]){
    if(argc<2){
        print_usage();
        return -1;
    }
    string folder_path(argv[1]);

    ROS_INFO("Start tracking");

    ros::init(argc, argv, "pose_estimator");
    Tracker tracker(folder_path);

    ros::spin();

    ROS_INFO("Stop tracking");
    
    return 0;
}
