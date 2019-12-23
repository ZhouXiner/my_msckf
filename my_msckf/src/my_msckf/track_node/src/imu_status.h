//
// Created by zhouxin on 2019/12/23.
//

#ifndef SRC_IMU_STATUS_H
#define SRC_IMU_STATUS_H

#include <map>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#define GRAVITY_ACCELERATION 9.81

struct IMUState{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef long long int StateIDType;

    StateIDType id;
    StateIDType  next_id;

    double time;

    //world->IMU(body)
    Eigen::Vector4d orientation;

    //IMU position in world
    Eigen::Vector3d position;

    //Veclocity IMU in world
    Eigen::Vector3d velocity;

    Eigen::Vector3d gyro_bias;
    Eigen::Vector3d acc_bias;

    Eigen::Matrix3d R_imu_cam0;
    Eigen::Vector3d t_cam0_imu;

    // These three variables should have the same physical
    // interpretation with `orientation`, `position`, and
    // `velocity`. There three variables are used to modify
    // the transition matrices to make the observability matrix
    // have proper null space.
    Eigen::Vector4d orientation_null;
    Eigen::Vector3d position_null;
    Eigen::Vector3d velocity_null;

    // Process noise
    double gyro_noise;
    double acc_noise;
    double gyro_bias_noise;
    double acc_bias_noise;

    // Gravity vector in the world frame
    static Eigen::Vector3d gravity;

    // Transformation offset from the IMU frame to
    // the body frame. The transformation takes a
    // vector from the IMU frame to the body frame.
    // The z axis of the body frame should point upwards.
    // Normally, this transform should be identity.
    static Eigen::Isometry3d T_imu_body;
    IMUState(): id(0), time(0),
                orientation(Eigen::Vector4d(0, 0, 0, 1)),
                position(Eigen::Vector3d::Zero()),
                velocity(Eigen::Vector3d::Zero()),
                gyro_bias(Eigen::Vector3d::Zero()),
                acc_bias(Eigen::Vector3d::Zero()),
                orientation_null(Eigen::Vector4d(0, 0, 0, 1)),
                position_null(Eigen::Vector3d::Zero()),
                velocity_null(Eigen::Vector3d::Zero()) {}

    IMUState(const StateIDType& new_id): id(new_id), time(0),
                orientation(Eigen::Vector4d(0, 0, 0, 1)),
                position(Eigen::Vector3d::Zero()),
                velocity(Eigen::Vector3d::Zero()),
                gyro_bias(Eigen::Vector3d::Zero()),
                acc_bias(Eigen::Vector3d::Zero()),
                orientation_null(Eigen::Vector4d(0, 0, 0, 1)),
                position_null(Eigen::Vector3d::Zero()),
                velocity_null(Eigen::Vector3d::Zero()) {}
};

typedef IMUState::StateIDType StateIDType;
#endif //SRC_IMU_STATUS_H
