#include "point_cloud_manager.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "PointCloudManagerNode");

//    PointCloudManager pc_manager("/VLP16_lidar_front/velodyne_points");
    PointCloudManager pc_manager("/drill_camera/camera/depth/color/points");

    ros::Rate r(100);
    while(ros::ok())
    {
        pc_manager.update();
        r.sleep();
    }
    return 0;
}
