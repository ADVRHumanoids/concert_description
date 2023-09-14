#include "point_cloud_manager.h"

#include <visualization_msgs/MarkerArray.h>
#include <eigen_conversions/eigen_msg.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>


PointCloudManager::PointCloudManager(std::string input_topic):
_point_cloud (new pcl::PointCloud<pcl::PointXYZ>),
_selected_point_cloud (new pcl::PointCloud<pcl::PointXYZ>),
_normals (new pcl::PointCloud<pcl::Normal>),
_frame_id("drill_camera_color_optical_frame"),
_nh("point_cloud_manager_node")
{
    _selected_point_cloud_sub = _nh.subscribe("/rviz_selected_points", 1, &PointCloudManager::selected_point_cloud_callback, this);
    _point_cloud_sub = _nh.subscribe(input_topic, 1, &PointCloudManager::point_cloud_callback, this);
    _clicked_point_sub = _nh.subscribe("/clicked_point", 1, &PointCloudManager::clicked_point_callback, this);
    _normal_marker_pub = _nh.advertise<visualization_msgs::Marker>("normal_marker", 10, true);
    _selected_point_cloud_publisher = _nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("selected_point_cloud", 1, true);
}

void PointCloudManager::update()
{
    if (_selected_point_cloud->empty())
    {
        ros::spinOnce();
        return;
    }

     compute_normals(0.25);
     average_normal_array();
     ros::spinOnce();
}

void PointCloudManager::transform_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, std::string frame_id)
{
    try
    {
        _listener.waitForTransform(frame_id, _frame_id, ros::Time(0), ros::Duration(10.0));
        _listener.lookupTransform(frame_id, _frame_id, ros::Time(0), _transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
    }

    Eigen::Affine3d b_T_cam;
    tf::transformTFToEigen(_transform, b_T_cam);
    pcl::transformPointCloud(*cloud_in, *cloud_out, b_T_cam.matrix());

    cloud_out->header.frame_id = frame_id;

    _frame_id = frame_id;
}

void PointCloudManager::selected_point_cloud_callback(const pcl::PointCloud<pcl::PointXYZ>::Ptr& msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc (new pcl::PointCloud<pcl::PointXYZ>);
    _selected_point_cloud = msg;

//    transform_point_cloud(pc, _selected_point_cloud, pc->header.frame_id);

    // Statistical outliers removal
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(_selected_point_cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh (1.0);
    sor.filter(*_selected_point_cloud);
    _selected_point_cloud_publisher.publish(_selected_point_cloud);
}


void PointCloudManager::point_cloud_callback(const pcl::PointCloud<pcl::PointXYZ>::Ptr& msg)
{
    _point_cloud = msg;
}

void PointCloudManager::clicked_point_callback(const geometry_msgs::PointStampedConstPtr &msg)
{
    _clicked_point = *msg;

    if(_frame_id != _clicked_point.header.frame_id)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc = _point_cloud;
        transform_point_cloud(pc, _point_cloud, _clicked_point.header.frame_id);
    }

    if (_point_cloud->empty())
    {
        return;
    }

    // create a PointCloud from the K nearest points around the clicked point
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(_point_cloud);

    pcl::PointXYZ pcl_clicked_point(msg->point.x, msg->point.y, msg->point.z);
    float radius = 0.03;

    std::vector<int> pointIdxRadiusSearch; //to store index of surrounding points
    std::vector<float> pointRadiusSquaredDistance; // to store distance to surrounding points

    if (kdtree.radiusSearch(pcl_clicked_point, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
    {
        for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
            std::cout << "    "  <<   _point_cloud->points[ pointIdxRadiusSearch[i] ].x
                    << " " << _point_cloud->points[ pointIdxRadiusSearch[i] ].y
                    << " " << _point_cloud->points[ pointIdxRadiusSearch[i] ].z
                    << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
    }

    for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
        _selected_point_cloud->points.push_back(_point_cloud->points[ pointIdxRadiusSearch[i] ]);
    _selected_point_cloud->header = _point_cloud->header;
    _selected_point_cloud->width = _selected_point_cloud->points.size ();
    _selected_point_cloud->height = 1;
    _selected_point_cloud->is_dense = true;

    _selected_point_cloud_publisher.publish(_selected_point_cloud);
}

void PointCloudManager::compute_normals(const double radius_search)
{
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(_selected_point_cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given)
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(tree);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch(radius_search);

    // Compute normals
    ne.compute(*_normals);
}

void PointCloudManager::from_normals_to_marker_array(std::string frame_id)
{
    // Dataset
    geometry_msgs::Point pnt_start, pnt_end;
    visualization_msgs::MarkerArray ma;

    std::string parent_frame;
    if (frame_id.empty())
        parent_frame = _selected_point_cloud->header.frame_id;
    else
        parent_frame = frame_id;

    ros::Time t = ros::Time::now();
    for ( int i = 0; i < _selected_point_cloud->width; i++ ) {

        // Assign values to Marker fields
        if ( !std::isnan (_normals->points[i].normal_x ) && !std::isnan (_normals->points[i].normal_y ) && !std::isnan (_normals->points[i].normal_z ) ) {
            visualization_msgs::Marker m;

            m.header.frame_id = parent_frame;
            m.header.stamp = t;
            m.id = i;
            m.action = visualization_msgs::Marker::ADD;
            m.type = visualization_msgs::Marker::ARROW;
            pnt_start.x = _selected_point_cloud->points[i].x;
            pnt_start.y = _selected_point_cloud->points[i].y;
            pnt_start.z = _selected_point_cloud->points[i].z;
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
            ma.markers.push_back(m);
        }
    }
}

void PointCloudManager::average_normal_array(std::string frame_id)
{
    // compute average point as origin point of the normal
    float sum_x = 0;
    float sum_y = 0;
    float sum_z = 0;

    for ( int i = 0; i < _selected_point_cloud->size(); i++ )
    {
        sum_x += _selected_point_cloud->points[i].x;
        sum_y += _selected_point_cloud->points[i].y;
        sum_z += _selected_point_cloud->points[i].z;
    }

    auto start = Eigen::Vector3d(sum_x / _selected_point_cloud->size(), sum_y / _selected_point_cloud->size(), sum_z / _selected_point_cloud->size());

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
        parent_frame = _selected_point_cloud->header.frame_id;
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

    broadcast_tf(Eigen::Vector3d(sumNormal_x / _normals->size(),
                                 sumNormal_y / _normals->size(),
                                 sumNormal_z / _normals->size()),
                 start);

    _normal_marker_pub.publish(m);
}

void PointCloudManager::broadcast_tf(Eigen::Vector3d normal, Eigen::Vector3d center)
{
    Eigen::Vector3d z_child = normal;

    Eigen::Vector3d x_child = Eigen::Vector3d(1, 0, 0) -  z_child(0) * z_child;
    x_child.normalize();

    Eigen::Vector3d y_child = z_child.cross(x_child);

    Eigen::Affine3d normal_T;
    normal_T.translation() = center;
    normal_T.linear().col(0) = x_child;
    normal_T.linear().col(1) = y_child;
    normal_T.linear().col(2) = z_child;

    tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::transformEigenToTF(normal_T, transform);

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), _frame_id, "normal_frame"));
}

