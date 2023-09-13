#include "point_cloud_manager.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "PointCloudManagerNode");

    PointCloudManager pc_manager("/rviz_selected_points");

    ros::Rate r(100);
    while(ros::ok())
    {
        pc_manager.update();
        r.sleep();
    }
    return 0;
}
