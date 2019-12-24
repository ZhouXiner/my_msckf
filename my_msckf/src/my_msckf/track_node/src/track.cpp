//
// Created by zhouxin on 2019/12/23.
//
#include "track.h"

using namespace Eigen;

//为全局变量提供初始值
StateIDType IMUState::next_id = 0;
double IMUState::gyro_noise = 0.001;
double IMUState::acc_noise = 0.01;
double IMUState::gyro_bias_noise = 0.001;
double IMUState::acc_bias_noise = 0.01;
Vector3d IMUState::gravity = Vector3d(0, 0, -GRAVITY_ACCELERATION);

Isometry3d IMUState::T_imu_body = Isometry3d::Identity();
Isometry3d CAMState::T_cam0_cam1 = Isometry3d::Identity();
FeatureIDType Feature::next_id = 0;
double Feature::observation_noise = 0.01;

MSCKF_VIO::MSCKF_VIO() {

}
Eigen::Isometry3d MSCKF_VIO::get_transfom(const vector<double> t_vec) {

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

bool MSCKF_VIO::load_parameters(ros::NodeHandle &nh){

    // Use variance instead of standard deviation.
    IMUState::gyro_noise *= IMUState::gyro_noise;
    IMUState::acc_noise *= IMUState::acc_noise;
    IMUState::gyro_bias_noise *= IMUState::gyro_bias_noise;
    IMUState::acc_bias_noise *= IMUState::acc_bias_noise;
    Feature::observation_noise *= Feature::observation_noise;

    state_server.imu_state.velocity = Vector3d(0.0,0.0,0.0);

    double gyro_bias_cov, acc_bias_cov, velocity_cov;
    gyro_bias_cov = 1e-4;
    velocity_cov = 0.25;
    acc_bias_cov = 1e-2;

    double extrinsic_rotation_cov, extrinsic_translation_cov;
    extrinsic_rotation_cov = 3.0462e-4;
    extrinsic_translation_cov = 1e-4;

    state_server.state_cov = MatrixXd::Zero(21,21);

    for (int i = 3; i < 6; ++i)
        state_server.state_cov(i, i) = gyro_bias_cov;
    for (int i = 6; i < 9; ++i)
        state_server.state_cov(i, i) = velocity_cov;
    for (int i = 9; i < 12; ++i)
        state_server.state_cov(i, i) = acc_bias_cov;
    for (int i = 15; i < 18; ++i)
        state_server.state_cov(i, i) = extrinsic_rotation_cov;
    for (int i = 18; i < 21; ++i)
        state_server.state_cov(i, i) = extrinsic_translation_cov;

    vector<double> T_imu_cam0_vec(16);
    nh.getParam("cam0/T_cam_imu",T_imu_cam0_vec);
    Isometry3d T_imu_cam0 = get_transfom(T_imu_cam0_vec);
    Isometry3d T_cam0_imu = T_imu_cam0.inverse();

    state_server.imu_state.R_imu_cam0 = T_cam0_imu.linear().transpose();
    state_server.imu_state.t_cam0_imu = T_cam0_imu.translation();

    vector<double> T_cam0_cam1_vec(16);
    nh.getParam("cam1/T_cn_cnm1",T_cam0_cam1_vec);
    CAMState::T_cam0_cam1 = get_transfom(T_cam0_cam1_vec);

    vector<double> T_imu_body_vec(16);
    nh.getParam("T_imu_body",T_imu_body_vec);

    IMUState::T_imu_body = get_transfom(T_imu_body_vec).inverse();

    ROS_INFO("===========================================");
    ROS_INFO("publish tf: %d", publish_tf);
    ROS_INFO("frame rate: %f", frame_rate);
    ROS_INFO("position std threshold: %f", position_std_threshold);
    ROS_INFO("Keyframe rotation threshold: %f", rotation_threshold);
    ROS_INFO("Keyframe translation threshold: %f", translation_threshold);
    ROS_INFO("Keyframe tracking rate threshold: %f", tracking_rate_threshold);
    ROS_INFO("gyro noise: %.10f", IMUState::gyro_noise);
    ROS_INFO("gyro bias noise: %.10f", IMUState::gyro_bias_noise);
    ROS_INFO("acc noise: %.10f", IMUState::acc_noise);
    ROS_INFO("acc bias noise: %.10f", IMUState::acc_bias_noise);
    ROS_INFO("observation noise: %.10f", Feature::observation_noise);
    ROS_INFO("initial velocity: %f, %f, %f",
             state_server.imu_state.velocity(0),
             state_server.imu_state.velocity(1),
             state_server.imu_state.velocity(2));
    ROS_INFO("initial gyro bias cov: %f", gyro_bias_cov);
    ROS_INFO("initial acc bias cov: %f", acc_bias_cov);
    ROS_INFO("initial velocity cov: %f", velocity_cov);
    ROS_INFO("initial extrinsic rotation cov: %f",
             extrinsic_rotation_cov);
    ROS_INFO("initial extrinsic translation cov: %f",
             extrinsic_translation_cov);

    cout << T_imu_cam0.linear() << endl;
    cout << T_imu_cam0.translation().transpose() << endl;

    ROS_INFO("max camera state #: %d", max_cam_state_size);
    ROS_INFO("===========================================");
    return true;
}

bool MSCKF_VIO::initilization(ros::NodeHandle &nh) {
    if(!load_parameters(nh)){
        return false;
    }
    ROS_INFO("Finish loading ROS parameters...");

    // Initialize state server
    state_server.continuous_noise_cov =
            Matrix<double, 12, 12>::Zero();
    state_server.continuous_noise_cov.block<3, 3>(0, 0) =
            Matrix3d::Identity()*IMUState::gyro_noise;
    state_server.continuous_noise_cov.block<3, 3>(3, 3) =
            Matrix3d::Identity()*IMUState::gyro_bias_noise;
    state_server.continuous_noise_cov.block<3, 3>(6, 6) =
            Matrix3d::Identity()*IMUState::acc_noise;
    state_server.continuous_noise_cov.block<3, 3>(9, 9) =
            Matrix3d::Identity()*IMUState::acc_bias_noise;

    for (int i = 1; i < 100; ++i) {
        boost::math::chi_squared chi_squared_dist(i);
        chi_squared_test_table[i] = boost::math::quantile(chi_squared_dist, 0.05);
    }

    return true;
}

void MSCKF_VIO::imu_track(const sensor_msgs::ImuConstPtr& msg){
    MSCKF_VIO::Imu_Msg_Buffer.push_back(*msg);
    if(!is_gravity_set){
        if(Imu_Msg_Buffer.size() < 200) return;
        initializeGravityAndBias();
        is_gravity_set = true;
    }
}

void MSCKF_VIO::initializeGravityAndBias() {
    Vector3d sum_angular_vel = Vector3d::Zero();
    Vector3d sum_linear_acc = Vector3d::Zero();

    for(const auto& imu_msg : Imu_Msg_Buffer){
        Vector3d angular_vel = Vector3d::Zero();
        Vector3d linear_acc = Vector3d::Zero();

        angular_vel[0] = double(imu_msg.angular_velocity.x);
        angular_vel[1] = double(imu_msg.angular_velocity.y);
        angular_vel[2] = double(imu_msg.angular_velocity.z);

        linear_acc[0] = double(imu_msg.linear_acceleration.x);
        linear_acc[1] = double(imu_msg.linear_acceleration.y);
        linear_acc[2] = double(imu_msg.linear_acceleration.z);

        sum_angular_vel += angular_vel;
        sum_linear_acc += linear_acc;
    }
    Imu_State.gyro_bias = sum_angular_vel / Imu_Msg_Buffer.size();

    Vector3d gravity_imu = sum_linear_acc / Imu_Msg_Buffer.size();

    double gravity_norm = gravity_imu.norm();

    Imu_State.gravity = Vector3d(0.0,0.0,gravity_norm);

    Quaterniond q0_i_w = Quaterniond::FromTwoVectors(gravity_imu,Imu_State.gravity);

    Imu_State.orientation = rotationToQuaternion(q0_i_w.toRotationMatrix().transpose());

    return;
}

void MSCKF_VIO::feature_track(const sensor_msgs::PointCloudConstPtr &feature_msg) {
    if(!is_gravity_set) return;

    if(!is_first_img){
        is_first_img = 1;
        state_server.imu_state.time = feature_msg->header.stamp.toSec();
    }

    static double max_processing_time = 0.0;
    static int critical_time_cntr = 0;
    double processing_start_time = ros::Time::now().toSec();
}