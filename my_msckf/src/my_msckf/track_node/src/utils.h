//
// Created by zhouxin on 2019/12/24.
//

#ifndef SRC_UTILS_H
#define SRC_UTILS_H

#include <cmath>
#include <Eigen/Dense>

inline Eigen::Isometry3d get_transfom(const std::vector<double> t_vec){
    Eigen::Matrix4d c;
    c << t_vec[0],t_vec[1],t_vec[2],t_vec[3],
            t_vec[4],t_vec[5],t_vec[6],t_vec[7],
            t_vec[8],t_vec[9],t_vec[10],t_vec[11],
            t_vec[12],t_vec[13],t_vec[14],t_vec[15];

    Eigen::Isometry3d T;

    T.linear()(0, 0)   = c(0, 0);
    T.linear()(0, 1)   = c(0, 1);
    T.linear()(0, 2)   = c(0, 2);
    T.linear()(1, 0)   = c(1, 0);
    T.linear()(1, 1)   = c(1, 1);
    T.linear()(1, 2)   = c(1, 2);
    T.linear()(2, 0)   = c(2, 0);
    T.linear()(2, 1)   = c(2, 1);
    T.linear()(2, 2)   = c(2, 2);
    T.translation()(0) = c(0, 3);
    T.translation()(1) = c(1, 3);
    T.translation()(2) = c(2, 3);
    return T;
}

inline Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d &w){
    Eigen::Matrix3d w_hat;
    w_hat(0, 0) = 0;
    w_hat(0, 1) = -w(2);
    w_hat(0, 2) = w(1);
    w_hat(1, 0) = w(2);
    w_hat(1, 1) = 0;
    w_hat(1, 2) = -w(0);
    w_hat(2, 0) = -w(1);
    w_hat(2, 1) = w(0);
    w_hat(2, 2) = 0;
    return w_hat;
}

inline void quaternionNormalize(Eigen::Vector4d& q){
    double norm = q.norm();
    q = q / norm;
    return;
}

inline Eigen::Matrix3d quaternionToRotation(const Eigen::Vector4d& q){
    const Eigen::Vector3d &v = q.block(0,0,3,1);
    const double s = q(3);
    Eigen::Matrix3d R =
            v * v.transpose() +
            s * s * Eigen::Matrix3d::Identity() +
            2 * s * skewSymmetric(v);
    return R;
}

inline Eigen::Vector4d rotationToQuaternion(const Eigen::Matrix3d &R){
    Eigen::Quaterniond q = Eigen::Quaterniond(R);
    q.normalize();
    Eigen::Vector4d q_vec;
    q_vec(0) = q.x();
    q_vec(1) = q.y();
    q_vec(2) = q.z();
    q_vec(3) = q.w();
    return q_vec;
}
#endif //SRC_UTILS_H
