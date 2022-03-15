#ifndef LIDAR_CAMERA_FUSION_H
#define LIDAR_CAMERA_FUSION_H

#define PCL_NO_PRECOMPILE

#include <iostream>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <pcl/common/common.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/point_cloud.h>

//#include <opencv-3.3.1-dev/opencv2/opencv.hpp>
#include <opencv2/opencv.hpp>

#include <rospy_tutorials/Floats.h>

#include <semantic_msg/bayes_msg.h>
#include <semantic_msg/max_msg.h>

#include "semantics_point_type.h"

class LidarCameraFusion {
public:
    LidarCameraFusion();

private:
    void InitROS();

    void ImageRawCallback(const sensor_msgs::Image::ConstPtr& image_msg);

    void CloudRawCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);

    void MaxSemanticCallback(const semantic_msg::max_msg::ConstPtr& max_semantic);

    void BayesSemanticCallback(const semantic_msg::bayes_msg::ConstPtr& bayes_semantic);

    tf::StampedTransform FindTransform(const std::string& target_frame, const std::string& source_frame);

    pcl::PointXYZ TransformPoint(const pcl::PointXYZ& in_point, const tf::StampedTransform& in_transform);

    void EucCluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud,
        std::vector<pcl::PointIndices>& cluster_indices,
        int cluster_tolerance,
        int min_cluster_size,
        int max_cluster_size);

private:
    ros::NodeHandle param_handle;
    ros::NodeHandle topic_handle;

    ros::Subscriber sub_image_raw;
    ros::Subscriber sub_cloud_raw;

    ros::Subscriber sub_max_semantic;
    ros::Subscriber sub_bayes_semantic;

    ros::Publisher pub_bayes_semantic_cloud;
    ros::Publisher pub_max_semantic_cloud;

    ros::Publisher pub_semantic_img;

private:
    std::string image_frame_id;
    cv::Mat image_frame;
    cv::Size image_size;

    sensor_msgs::PointCloud2 cloud_frame;

    // Robosense 雷达和 ZED 相机外参
    cv::Mat camera_extrinsic_mat;
    bool camera_extrinsic_mat_ok;

    // 外参逆矩阵
    cv::Mat camera_extrinsic_mat_inv;

    // ZED 相机内参
    cv::Mat camera_instrinsics_mat;
    bool camera_instrinsics_mat_ok;
    float fx, fy, cx, cy;

    // ZED 相机畸变矩阵
    cv::Mat distortion_coefficients;
    // ZED 相机畸变模型
    std::string dist_model;

private:
    // Max 语义图像
    cv::Mat max_frame;

    // 3 组 Bayes 语义图像
    cv::Mat bayes_frame_1;
    cv::Mat bayes_frame_2;
    cv::Mat bayes_frame_3;

private:
    // 定义相机和雷达之间的坐标转换关系
    tf::StampedTransform camera_lidar_tf;

    // tf 坐标监听者
    tf::TransformListener transform_listener;

    // 判断是否找到雷达和相机之间的坐标转换关系
    bool camera_lidar_tf_ok;

private:
    static const std::string kNodeName;
};

#endif // LIDAR_CAMERA_FUSION_H
