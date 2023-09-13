#include "point_cloud_manager.h"

#include <visualization_msgs/MarkerArray.h>
#include <eigen_conversions/eigen_msg.h>


#include <tf_conversions/tf_eigen.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>


PointCloudManager::PointCloudManager(std::string input_topic):
_point_cloud (new pcl::PointCloud<pcl::PointXYZ>),
_normals (new pcl::PointCloud<pcl::Normal>),
_nh("point_cloud_manager_node")
{
    _point_cloud_sub = _nh.subscribe(input_topic, 1, &PointCloudManager::point_cloud_callback, this);
    _normals_pub = _nh.advertise<pcl::PointCloud<pcl::Normal>>("normals", 10, true);
    _normal_marker_pub = _nh.advertise<visualization_msgs::Marker>("normal_marker", 10, true);
}

void PointCloudManager::update()
{
    if (_point_cloud->empty())
    {
        ros::spinOnce();
        return;
    }

     compute_normals(0.25);
     average_normal_array();
     ros::spinOnce();
}

void PointCloudManager::point_cloud_callback(const pcl::PointCloud<pcl::PointXYZ>::Ptr& msg)
{
    try
    {
        _listener.waitForTransform("base_link", "VLP16_lidar_front", ros::Time(0), ros::Duration(10.0));
        _listener.lookupTransform("base_link", "VLP16_lidar_front", ros::Time(0), _transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
    }

    Eigen::Affine3d b_T_cam;
    tf::transformTFToEigen(_transform, b_T_cam);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc (new pcl::PointCloud<pcl::PointXYZ>);
    pc = msg;

    // Statistical outliers removal
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(pc);
    sor.setMeanK(50);
    sor.setStddevMulThresh (1.0);
    sor.filter(*pc);

    pcl::transformPointCloud(*pc, *_point_cloud, b_T_cam.matrix());

    _point_cloud->header.frame_id = "base_link";

}

void PointCloudManager::compute_normals(const double radius_search)
{
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (_point_cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given)
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(tree);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch(radius_search);

    // Compute normals
    ne.compute(*_normals);

    _normals_pub.publish(_normals);
}

void PointCloudManager::from_normals_to_marker_array(std::string frame_id)
{
    // Dataset
    geometry_msgs::Point pnt_start, pnt_end;
    visualization_msgs::MarkerArray ma;

    std::string parent_frame;
    if (frame_id.empty())
        parent_frame = _point_cloud->header.frame_id;
    else
        parent_frame = frame_id;

    ros::Time t = ros::Time::now();
    for ( int i = 0; i < _point_cloud->width; i++ ) {

        // Assign values to Marker fields
        if ( !std::isnan (_normals->points[i].normal_x ) && !std::isnan (_normals->points[i].normal_y ) && !std::isnan (_normals->points[i].normal_z ) ) {
            visualization_msgs::Marker m;

            m.header.frame_id = parent_frame;
            m.header.stamp = t;
            m.id = i;
            m.action = visualization_msgs::Marker::ADD;
            m.type = visualization_msgs::Marker::ARROW;
            pnt_start.x = _point_cloud->points[i].x;
            pnt_start.y = _point_cloud->points[i].y;
            pnt_start.z = _point_cloud->points[i].z;
            m.points.push_back ( pnt_start );

            pnt_end.x = pnt_start.x + _normals->points[i].normal_x;
            pnt_end.y = pnt_start.y + _normals->points[i].normal_y;
            pnt_end.z = pnt_start.z + _normals->points[i].normal_z;
            m.points.push_back ( pnt_end );
            m.scale.x = 0.01;
            m.scale.y = 0.02;
            m.scale.z = 0.02;
            m.color.r = 255;
            m.color.g = 0;
            m.color.b = 0;
            m.color.a = 1;

            // Assign Marker to MarkerArray
            ma.markers.push_back ( m );
        }
    }
}

void PointCloudManager::average_normal_array(std::string frame_id)
{
    // compute average point as origin point of the normal
    float sum_x = 0;
    float sum_y = 0;
    float sum_z = 0;

    for ( int i = 0; i < _point_cloud->size(); i++ ) {
        sum_x += _point_cloud->points[i].x;
        sum_y += _point_cloud->points[i].y;
        sum_z += _point_cloud->points[i].z;
    }

    auto start = Eigen::Vector3d(sum_x / _point_cloud->size(), sum_y / _point_cloud->size(), sum_z / _point_cloud->size());

    float sumNormal_x = 0;
    float sumNormal_y = 0;
    float sumNormal_z = 0;


    for ( int i = 0; i < _normals->size(); i++ ) {
        if ( !std::isnan ( _normals->points[i].normal_x ) ) {
            sumNormal_x += _normals->points[i].normal_x;
        }
        if ( !std::isnan ( _normals->points[i].normal_y ) ) {
            sumNormal_y += _normals->points[i].normal_y;
        }
        if ( !std::isnan ( _normals->points[i].normal_z ) ) {
            sumNormal_z += _normals->points[i].normal_z;
        }
    }

    auto end = Eigen::Vector3d(start(0) + sumNormal_x / _normals->size(),
                                start(1) + sumNormal_y / _normals->size(),
                                start(2) + sumNormal_z / _normals->size());

    std::string parent_frame;
    if (frame_id.empty())
        parent_frame = _point_cloud->header.frame_id;
    else
        parent_frame = frame_id;

    visualization_msgs::Marker m;
    m.header.frame_id = parent_frame;
    m.header.stamp = ros::Time::now();
    m.id = 0;
    m.action = visualization_msgs::Marker::ADD;
    m.type = visualization_msgs::Marker::ARROW;

    geometry_msgs::Point pnt_start, pnt_end;
    tf::pointEigenToMsg(start, pnt_start);
    m.points.push_back ( pnt_start );

    tf::pointEigenToMsg(end, pnt_end);
    m.points.push_back ( pnt_end );

    m.scale.x = 0.01;
    m.scale.y = 0.02;
    m.scale.z = 0.02;
    m.color.r = 255;
    m.color.g = 0;
    m.color.b = 0;
    m.color.a = 1;

    broadcast_tf(Eigen::Vector3d(end));

    _normal_marker_pub.publish(m);
}

void PointCloudManager::broadcast_tf(Eigen::Vector3d normal, std::string frame_id)
{
    Eigen::Vector3d x_parent(1, 0, 0), y_parent (0, 1, 0), z_parent(0, 0, 1);
    Eigen::Vector3d z_child = normal;



}

