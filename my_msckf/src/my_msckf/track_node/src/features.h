//
// Created by zhouxin on 2019/12/23.
//

#ifndef SRC_FEATURES_H
#define SRC_FEATURES_H

#include <iostream>
#include <map>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include "imu_status.h"
#include "cam_status.h"
#include  "utils.h"

struct Feature{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef long long int FeatureIDType;

    struct OptimizationConfig {
        double translation_threshold;
        double huber_epsilon;
        double estimation_precision;
        double initial_damping;
        int outer_loop_max_iteration;
        int inner_loop_max_iteration;

        OptimizationConfig() :
                translation_threshold(0.2),
                huber_epsilon(0.01),
                estimation_precision(5e-7),
                initial_damping(1e-3),
                outer_loop_max_iteration(10),
                inner_loop_max_iteration(10) {
            return;
        }
    };

    // Optimization configuration for solving the 3d position.
    // Constructors for the struct.
    Feature(): id(0), position(Eigen::Vector3d::Zero()),
               is_initialized(false) {}

    Feature(const FeatureIDType& new_id): id(new_id),
                                          position(Eigen::Vector3d::Zero()),
                                          is_initialized(false) {}

    inline bool checkMotion(
            const CamStateServer& cam_states) const;

    inline void generateInitialGuess(
            const Eigen::Isometry3d& T_c1_c2, const Eigen::Vector2d& z1,
            const Eigen::Vector2d& z2, Eigen::Vector3d& p) const;

    inline void cost(const Eigen::Isometry3d& T_c0_ci,
                     const Eigen::Vector3d& x, const Eigen::Vector2d& z,
                     double& e) const;

    //利用Levenberg-Marquart method求解地图点位置，如果位置性质差则认为无法初始化
    //显然，我们可以直接用svd代替这个过程，根据svd的结果的效果决定即可
    inline bool initializePosition(
            const CamStateServer& cam_states);

    inline void jacobian(const Eigen::Isometry3d& T_c0_ci,
                         const Eigen::Vector3d& x, const Eigen::Vector2d& z,
                         Eigen::Matrix<double, 2, 3>& J, Eigen::Vector2d& r,
                         double& w) const;

    std::map<StateIDType, Eigen::Vector4d, std::less<StateIDType>,
    Eigen::aligned_allocator<
            std::pair<const StateIDType, Eigen::Vector4d> > > observations;

    // 3d postion of the feature in the world frame.
    StateIDType id;
    // id for next feature
    static FeatureIDType next_id;
    Eigen::Vector3d position;

    // A indicator to show if the 3d postion of the feature
    // has been initialized or not.
    bool is_initialized;

    // Noise for a normalized feature measurement.
    static double observation_noise;

    // Optimization configuration for solving the 3d position.
    static OptimizationConfig optimization_config;

};

typedef Feature::FeatureIDType FeatureIDType;
typedef std::map<FeatureIDType, Feature, std::less<int>,
        Eigen::aligned_allocator<
                std::pair<const FeatureIDType, Feature> > > MapServer;

bool Feature::checkMotion(const CamStateServer &cam_states) const{
    const FeatureIDType start_cam_id = observations.begin()->first;
    const FeatureIDType end_cam_id = (observations.end()--)->first;

    Eigen::Isometry3d first_cam_pose;
    first_cam_pose.linear() = quaternionToRotation(
            cam_states.find(start_cam_id)->second.orientation).transpose();
    first_cam_pose.translation() = cam_states.find(start_cam_id)->second.position;

    Eigen::Isometry3d end_cam_pose;
    end_cam_pose.linear() = quaternionToRotation(
            cam_states.find(end_cam_id)->second.orientation).transpose();
    end_cam_pose.translation() = cam_states.find(end_cam_id)->second.position;

    Eigen::Vector3d feature_direction(
            observations.begin()->second(0),observations.begin()->second(1),1.0);

    feature_direction = feature_direction / feature_direction.norm();
    feature_direction = first_cam_pose.linear() * feature_direction;

    Eigen::Vector3d translation = first_cam_pose.translation() - end_cam_pose.translation();
    double parallel_translation =
            translation.transpose()*feature_direction;
    Eigen::Vector3d orthogonal_translation = translation -
                                             parallel_translation*feature_direction;

    if (orthogonal_translation.norm() >
        optimization_config.translation_threshold)
        return true;
    else return false;
}

bool Feature::initializePosition(const CamStateServer &cam_states) {

    std::vector<Eigen::Isometry3d,
    Eigen::aligned_allocator<Eigen::Isometry3d>> cam_poses;
    std::vector<Eigen::Vector2d,
    Eigen::aligned_allocator<Eigen::Vector2d>> measurements;

    for(auto &m : observations){
        auto cam_state_iter = cam_states.find(m.first);

        if(cam_state_iter == cam_states.end()) continue;

         measurements.push_back(m.second.head<2>());
         measurements.push_back(m.second.tail<2>());

        Eigen::Isometry3d cam0_pose;
        cam0_pose.linear() = quaternionToRotation(
                cam_state_iter->second.orientation).transpose();
        cam0_pose.translation() = cam_state_iter->second.position;

        Eigen::Isometry3d cam1_pose;
        cam1_pose = cam0_pose * CAMState::T_cam0_cam1.inverse();

        cam_poses.push_back(cam0_pose);
        cam_poses.push_back(cam1_pose);
    }

    Eigen::Isometry3d T_c0_w = cam_poses[0];
    for(auto &pose:cam_poses){
        pose = pose.inverse() * T_c0_w;
    }

    Eigen::Vector3d  initial_position(0.0,0.0,0.0);
    generateInitialGuess(cam_poses[cam_poses.size() - 1],measurements[0],
            measurements[measurements.size() - 1],initial_position);

    Eigen::Vector3d solution(
            initial_position(0)/initial_position(2),
            initial_position(1)/initial_position(2),
            1.0/initial_position(2));

    // Apply Levenberg-Marquart method to solve for the 3d position.
    double lambda = optimization_config.initial_damping;
    int inner_loop_cntr = 0;
    int outer_loop_cntr = 0;
    bool is_cost_reduced = false;
    double delta_norm = 0;

    // Compute the initial cost.
    double total_cost = 0.0;
    for (int i = 0; i < cam_poses.size(); ++i) {
        double this_cost = 0.0;
        cost(cam_poses[i], solution, measurements[i], this_cost);
        total_cost += this_cost;
    }

    // Outer loop.
    do {
        Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
        Eigen::Vector3d b = Eigen::Vector3d::Zero();

        for (int i = 0; i < cam_poses.size(); ++i) {
            Eigen::Matrix<double, 2, 3> J;
            Eigen::Vector2d r;
            double w;

            jacobian(cam_poses[i], solution, measurements[i], J, r, w);

            if (w == 1) {
                A += J.transpose() * J;
                b += J.transpose() * r;
            } else {
                double w_square = w * w;
                A += w_square * J.transpose() * J;
                b += w_square * J.transpose() * r;
            }
        }

        // Inner loop.
        // Solve for the delta that can reduce the total cost.
        do {
            Eigen::Matrix3d damper = lambda * Eigen::Matrix3d::Identity();
            Eigen::Vector3d delta = (A+damper).ldlt().solve(b);
            Eigen::Vector3d new_solution = solution - delta;
            delta_norm = delta.norm();

            double new_cost = 0.0;
            for (int i = 0; i < cam_poses.size(); ++i) {
                double this_cost = 0.0;
                cost(cam_poses[i], new_solution, measurements[i], this_cost);
                new_cost += this_cost;
            }

            if (new_cost < total_cost) {
                is_cost_reduced = true;
                solution = new_solution;
                total_cost = new_cost;
                lambda = lambda/10 > 1e-10 ? lambda/10 : 1e-10;
            } else {
                is_cost_reduced = false;
                lambda = lambda*10 < 1e12 ? lambda*10 : 1e12;
            }

        } while (inner_loop_cntr++ <
                 optimization_config.inner_loop_max_iteration && !is_cost_reduced);

        inner_loop_cntr = 0;

    } while (outer_loop_cntr++ <
             optimization_config.outer_loop_max_iteration &&
             delta_norm > optimization_config.estimation_precision);

    // Covert the feature position from inverse depth
    // representation to its 3d coordinate.
    Eigen::Vector3d final_position(solution(0)/solution(2),
                                   solution(1)/solution(2), 1.0/solution(2));

    // Check if the solution is valid. Make sure the feature
    // is in front of every camera frame observing it.
    bool is_valid_solution = true;
    for (const auto& pose : cam_poses) {
        Eigen::Vector3d position =
                pose.linear()*final_position + pose.translation();
        if (position(2) <= 0) {
            is_valid_solution = false;
            break;
        }
    }

    // Convert the feature position to the world frame.
    position = T_c0_w.linear()*final_position + T_c0_w.translation();

    if (is_valid_solution)
        is_initialized = true;

    return is_valid_solution;
}


void Feature::generateInitialGuess(
        const Eigen::Isometry3d& T_c1_c2, const Eigen::Vector2d& z1,
        const Eigen::Vector2d& z2, Eigen::Vector3d& p) const {
    // Construct a least square problem to solve the depth.
    Eigen::Vector3d m = T_c1_c2.linear() * Eigen::Vector3d(z1(0), z1(1), 1.0);

    Eigen::Vector2d A(0.0, 0.0);
    A(0) = m(0) - z2(0)*m(2);
    A(1) = m(1) - z2(1)*m(2);

    Eigen::Vector2d b(0.0, 0.0);
    b(0) = z2(0)*T_c1_c2.translation()(2) - T_c1_c2.translation()(0);
    b(1) = z2(1)*T_c1_c2.translation()(2) - T_c1_c2.translation()(1);

    // Solve for the depth.
    //depth实际上求得就是1/z，所以这里得到的点也已经是在cam0平面上的归一化坐标了
    double depth = (A.transpose() * A).inverse() * A.transpose() * b;
    p(0) = z1(0) * depth;
    p(1) = z1(1) * depth;
    p(2) = depth;
    return;
}

void Feature::cost(const Eigen::Isometry3d &T_c0_ci, const Eigen::Vector3d &x, const Eigen::Vector2d &z,
                   double &e) const {
    // Compute hi1, hi2, and hi3 as Equation (37).
    const double& alpha = x(0);
    const double& beta = x(1);
    const double& rho = x(2);

    Eigen::Vector3d h = T_c0_ci.linear()*
                        Eigen::Vector3d(alpha, beta, 1.0) + rho*T_c0_ci.translation();
    double& h1 = h(0);
    double& h2 = h(1);
    double& h3 = h(2);

    // Predict the feature observation in ci frame.
    Eigen::Vector2d z_hat(h1/h3, h2/h3);

    // Compute the residual.
    e = (z_hat-z).squaredNorm();
    return;
}

void Feature::jacobian(const Eigen::Isometry3d& T_c0_ci,
                       const Eigen::Vector3d& x, const Eigen::Vector2d& z,
                       Eigen::Matrix<double, 2, 3>& J, Eigen::Vector2d& r,
                       double& w) const {

    // Compute hi1, hi2, and hi3 as Equation (37).
    const double& alpha = x(0);
    const double& beta = x(1);
    const double& rho = x(2);

    Eigen::Vector3d h = T_c0_ci.linear()*
                        Eigen::Vector3d(alpha, beta, 1.0) + rho*T_c0_ci.translation();
    double& h1 = h(0);
    double& h2 = h(1);
    double& h3 = h(2);

    // Compute the Jacobian.
    Eigen::Matrix3d W;
    W.leftCols<2>() = T_c0_ci.linear().leftCols<2>();
    W.rightCols<1>() = T_c0_ci.translation();

    J.row(0) = 1/h3*W.row(0) - h1/(h3*h3)*W.row(2);
    J.row(1) = 1/h3*W.row(1) - h2/(h3*h3)*W.row(2);

    // Compute the residual.
    Eigen::Vector2d z_hat(h1/h3, h2/h3);
    r = z_hat - z;

    // Compute the weight based on the residual.
    double e = r.norm();
    if (e <= optimization_config.huber_epsilon)
        w = 1.0;
    else
        w = std::sqrt(2.0*optimization_config.huber_epsilon / e);

    return;
}

#endif //SRC_FEATURES_H
