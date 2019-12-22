//
// Created by zhouxin on 2019/12/21.
//

#ifndef SRC_TRACK_NODE_H
#define SRC_TRACK_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <map>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <Eigen/QR>
#include <Eigen/SparseCore>
#include <opencv2/opencv.hpp>
#include <opencv2/video.hpp>
#include <opencv2/core/eigen.hpp>

#include <msckf/camera.h>
#include <msckf/feature.h>
#include <msckf/imu.h>
#include <msckf/math_utils.h>
#include <msckf/image_node.h>
#include <msckf/CameraMeasurement.h>


using namespace std;
using namespace Eigen;


vector<sensor_msgs::Imu> Track_Imu_Msg_Buffer;

ros::Subscriber track_imu_sub;
ros::Subscriber feature_sub;

void track_imu_callback(const sensor_msgs::ImuConstPtr& msg);
void track_feature_callback(const msckf::CameraMeasurementConstPtr& msg);

void initializeGravityAndBias();

#endif //SRC_TRACK_NODE_H
