//
// Created by zhouxin on 2019/12/7.
//

#ifndef SRC_IMU_H
#define SRC_IMU_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;

struct Imu_Status{
    int id;
    Vector4d orientation;
    Vector3d position;
    Vector3d velocity;

    // Bias for measured angular velocity
    // and acceleration.
    Vector3d gyro_bias;
    Vector3d acc_bias;

    // Process noise
    static double gyro_noise;
    static double acc_noise;
    static double gyro_bias_noise;
    static double acc_bias_noise;

    // Gravity vector in the world frame
    Vector3d gravity;
    Isometry3d T_imu_body;

    Matrix3d R_imu_cam0;
    Vector3d t_cam0_imu;


};

#endif //SRC_IMU_H
