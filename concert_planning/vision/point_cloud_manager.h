#ifndef POINTCLOUDMANAGER_H
#define POINTCLOUDMANAGER_H

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

class PointCloudManager
{
public:
    PointCloudManager(std::string input_topic);

    void update();

private:
    void point_cloud_callback(const pcl::PointCloud<pcl::PointXYZ>::Ptr& msg);

    void compute_normals(const double radius_search);
    void from_normals_to_marker_array(std::string frame_id = "");
    void average_normal_array(std::string frame_id = "");
    void broadcast_tf(Eigen::Vector3d normal, std::string frame_id = "");

    ros::NodeHandle _nh;
    ros::Subscriber _point_cloud_sub;
    ros::Publisher _normals_pub;
    ros::Publisher _normal_marker_pub;

    pcl::PointCloud<pcl::PointXYZ>::Ptr _point_cloud;
    pcl::PointCloud<pcl::Normal>::Ptr _normals;

    tf::TransformListener _listener;
    tf::StampedTransform _transform;
    tf::TransformBroadcaster _broadcaster;
};

#endif // POINTCLOUDMANAGER_H
