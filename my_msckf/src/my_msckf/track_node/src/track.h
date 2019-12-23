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

using namespace std;

class MSCKF_VIO{

public:
    MSCKF_VIO();

    vector<sensor_msgs::Imu> Imu_Msg_Buffer(0);

    struct StateServer{
        IMUState imu_state;
        CamStateServer cam_states;
        Eigen::MatrixXd state_cov;
        Eigen::Matrix<double,12,12> continuous_noise_cov;
    };

    StateServer state_server;
    IMUState static_imu_state;
    CAMState static_cam_state;

    bool initialization();

    void imu_track(const sensor_msgs::ImuConstPtr& msg);
};

#endif //SRC_TRACK_NODE_H
