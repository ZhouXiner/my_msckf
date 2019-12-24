//
// Created by zhouxin on 2019/12/23.
//
#pragma once
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

#include "imu_status.h"
#include "cam_status.h"
#include "features.h"

using namespace std;

class MSCKF_VIO{

public:
    MSCKF_VIO();

    bool load_parameters(ros::NodeHandle &nh);
    Eigen::Isometry3d get_transfom(const vector<double> t_vec);
    bool initilization(ros::NodeHandle &nh);
    void imu_track(const sensor_msgs::ImuConstPtr& msg);
    void initializeGravityAndBias();
    void feature_track(const sensor_msgs::PointCloudConstPtr &feature_msg);

    vector<sensor_msgs::Imu> Imu_Msg_Buffer;

    struct StateServer{
        IMUState imu_state;
        CamStateServer cam_states;
        Eigen::MatrixXd state_cov;
        Eigen::Matrix<double,12,12> continuous_noise_cov;
    };
    StateServer state_server;

    // Chi squared test table.
    static std::map<int, double> chi_squared_test_table;

    bool is_gravity_set = 0;
    bool is_first_img = 0;

    // The position uncertainty threshold is used to determine
    // when to reset the system online. Otherwise, the ever-
    // increaseing uncertainty will make the estimation unstable.
    // Note this online reset will be some dead-reckoning.
    // Set this threshold to nonpositive to disable online reset.
    double position_std_threshold = 8.0;

    // Tracking rate
    double tracking_rate;

    // Threshold for determine keyframes
    double translation_threshold = 0.4;
    double rotation_threshold = 0.2618;
    double tracking_rate_threshold = 0.5;

    // Whether to publish tf or not.
    bool publish_tf = true;

    // Framte rate of the stereo images. This variable is
    // only used to determine the timing threshold of
    // each iteration of the filter.
    double frame_rate = 20;

    // Maximum number of camera states
    int max_cam_state_size = 20;

};

#endif //SRC_TRACK_NODE_H
