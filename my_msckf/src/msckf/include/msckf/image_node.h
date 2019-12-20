//
// Created by zhouxin on 2019/12/6.
//

#ifndef SRC_IMAGE_NODE_H
#define SRC_IMAGE_NODE_H

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

using namespace std;
using namespace Eigen;



vector<sensor_msgs::Imu> Imu_Msg_Buffer;
vector<Camera_Status> Camera_Buffer;
vector<Feature_Status> Curr_Feature_Buffer;
vector<Feature_Status> Pre_Feature_Buffer;
Imu_Status Imu_State;

cv_bridge::CvImageConstPtr cam0_prev_img_ptr;
cv_bridge::CvImageConstPtr cam0_curr_img_ptr;
cv_bridge::CvImageConstPtr cam1_curr_img_ptr;


// Camera calibration parameters
std::string cam0_distortion_model;
Vector2i cam0_resolution;
Vector4d cam0_intrinsics;
Vector4d cam0_distortion_coeffs;

std::string cam1_distortion_model;
Vector2i cam1_resolution;
Vector4d cam1_intrinsics;
Vector4d cam1_distortion_coeffs;



// Take a vector from cam0 frame to the IMU frame.
Matrix3d R_cam0_imu;
Vector3d t_cam0_imu;
// Take a vector from cam1 frame to the IMU frame.
Matrix3d R_cam1_imu;
Vector3d t_cam1_imu;

vector<cv::Point2f> curr_cam0_points;
vector<cv::Point2f> curr_cam1_points;
vector<cv::Point2f> prev_cam0_points;
vector<cv::Point2f> prev_cam1_points;
vector<int> track_id;

int first_image_in = 0;
int fast_threshold = 200;
long long int feature_id = 0;
long long int camera_id = 0;

void imu_callback(const sensor_msgs::ImuConstPtr& msg);

void stereo_callback(const sensor_msgs::ImageConstPtr& cam0_msg,
                     const sensor_msgs::ImageConstPtr& cam1_msg);
bool initialize(ros::NodeHandle &nh);
bool load_parameter(ros::NodeHandle &nh);
void initialize_feature_track();
void undistort_points(const vector<cv::Point2f>& pts_in,vector<cv::Point2f>& pts_out,
                          const Vector4d &intrinsics,const Vector4d &distortion_coeffs);
void predict_track_points(vector<cv::Point2f> &pt_predict);
void predict_theta_R(Matrix3d &R_theta_c);
void feature_track();
void add_new_feature();
void status_change();

#endif
