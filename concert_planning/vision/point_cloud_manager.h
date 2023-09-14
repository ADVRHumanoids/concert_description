#ifndef POINTCLOUDMANAGER_H
#define POINTCLOUDMANAGER_H

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/PointStamped.h>

class PointCloudManager
{
public:
    PointCloudManager(std::string input_topic);

    void update();

private:
    void point_cloud_callback(const pcl::PointCloud<pcl::PointXYZ>::Ptr& msg);
    void selected_point_cloud_callback(const pcl::PointCloud<pcl::PointXYZ>::Ptr& msg);
    void clicked_point_callback(const geometry_msgs::PointStampedConstPtr& msg);

    void compute_normals(const double radius_search);
    void from_normals_to_marker_array(std::string frame_id = "");
    void average_normal_array(std::string frame_id = "");
    void broadcast_tf(Eigen::Vector3d normal,
                      Eigen::Vector3d center);
    void transform_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out,
                               std::string frame_id);

    ros::NodeHandle _nh;
    ros::Subscriber _point_cloud_sub, _selected_point_cloud_sub;
    ros::Subscriber _clicked_point_sub;
    ros::Publisher _normal_marker_pub;
    ros::Publisher _selected_point_cloud_publisher;

    pcl::PointCloud<pcl::PointXYZ>::Ptr _point_cloud, _selected_point_cloud;
    pcl::PointCloud<pcl::Normal>::Ptr _normals;

    std::string _frame_id;
    geometry_msgs::PointStamped _clicked_point;

    tf::TransformListener _listener;
    tf::StampedTransform _transform;
    tf::TransformBroadcaster _broadcaster;
};

#endif // POINTCLOUDMANAGER_H
