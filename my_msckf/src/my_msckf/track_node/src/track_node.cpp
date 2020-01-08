#include "features.h"
#include "utils.h"
#include "imu_status.h"

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <boost/math/distributions/chi_squared.hpp>
#include <map>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <Eigen/QR>
#include <Eigen/SparseCore>
#include <Eigen/SPQRSupport>

#include <opencv2/core/eigen.hpp>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

using namespace std;
using namespace Eigen;

ros::Publisher pub_vio_path;
ros::Publisher pub_key_odometrys;

bool load_parameters(ros::NodeHandle nh);
bool initilization(ros::NodeHandle &nh);
void imu_track(const sensor_msgs::Imu &msg);
void initializeGravityAndBias();
void propogate_process(const double& time);
void propogate_model(const double& time,
                     const Eigen::Vector3d& m_gyro,
                     const Eigen::Vector3d& m_acc);
void state_augmentation(const double& time);
void addFeatureObservations(const sensor_msgs::PointCloudConstPtr &feature_msg_cloud);
void removeLostFeatures();
void featureJacobian(const FeatureIDType& feature_id,
                     const std::vector<StateIDType>& cam_state_ids,
                     Eigen::MatrixXd& H_x, Eigen::VectorXd& r);
void measurementJacobian(const StateIDType& cam_state_id,
                         const FeatureIDType& feature_id,
                         Eigen::Matrix<double, 4, 6>& H_x,
                         Eigen::Matrix<double, 4, 3>& H_f,
                         Eigen::Vector4d& r);
bool gatingTest(const MatrixXd& H, const VectorXd& r, const int& dof);
void measurementUpdate(const MatrixXd& H, const VectorXd& r);
void pruneCamStateBuffer();
void findRemoveCameraids(std::vector<StateIDType>& rm_cam_ids);
void feature_track();
void predictNewState(const double& dt,const Vector3d& gyro,const Vector3d& acc);
void publish(const ros::Time& time);
void onlineReset();

std::vector<sensor_msgs::Imu> Imu_Msg_Buffer;


struct StateServer{
    IMUState imu_state;
    CamStateServer cam_states;
    Eigen::MatrixXd state_cov;
    Eigen::Matrix<double,12,12> continuous_noise_cov;
};
StateServer state_server;
MapServer map_server;

// Chi squared test table.
std::map<int, double> chi_squared_test_table;

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

int iter_num = 0;

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
Feature::OptimizationConfig Feature::optimization_config;

bool load_parameters(ros::NodeHandle nh){
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

//这里的命名规则值得注意：R_imu_cam0:表示的就是从imu到cam0的转变
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

    ROS_INFO("max camera state #: %d", max_cam_state_size);
    ROS_INFO("===========================================");
    return true;
}



bool initilization(ros::NodeHandle &nh) {
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
//随着卡方分布的自由度增加，卡方分布也在变换
        boost::math::chi_squared chi_squared_dist(i);

//对于不同的卡方分布，阈值不一样
        chi_squared_test_table[i] = boost::math::quantile(chi_squared_dist, 0.05);
    }
   return true;
}


void initializeGravityAndBias() {
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
    state_server.imu_state.gyro_bias = sum_angular_vel / Imu_Msg_Buffer.size();

    Vector3d gravity_imu = sum_linear_acc / Imu_Msg_Buffer.size();

    double gravity_norm = gravity_imu.norm();

    IMUState::gravity = Vector3d(0.0,0.0,gravity_norm);

    Quaterniond q0_i_w = Quaterniond::FromTwoVectors(gravity_imu,IMUState::gravity);

    state_server.imu_state.orientation = rotationToQuaternion(q0_i_w.toRotationMatrix().transpose());

    return;
}


void imu_callback(const sensor_msgs::ImuConstPtr msg){
    Imu_Msg_Buffer.push_back(* msg);
    if(!is_gravity_set){
        if(Imu_Msg_Buffer.size() < 200) return;
        initializeGravityAndBias();
        is_gravity_set = true;
    }
    return;
}

void predictNewState(const double &dt, const Vector3d &gyro, const Vector3d &acc) {
    //当前时刻的PVQ
    Vector4d& q = state_server.imu_state.orientation;
    Vector3d& p = state_server.imu_state.position;
    Vector3d& v = state_server.imu_state.velocity;

    Matrix4d Omega = Matrix4d::Zero();
    Omega.block<3,3>(0,0) = -skewSymmetric(gyro);
    Omega.block<3,1>(0,3) = gyro;
    Omega.block<1,3>(3,0) = -gyro;
    double gyro_norm = q.norm();

    Vector4d qt_dt,qt_ddt;
    if(gyro_norm > 10e-5){
        qt_dt = (cos(gyro_norm * dt * 0.5)*Matrix4d::Identity() +
                 (1 / gyro_norm)*sin(gyro_norm * dt * 0.5)*Omega)*q;
        qt_ddt = (cos(gyro_norm * dt * 0.25)*Matrix4d::Identity() +
                  (1 / gyro_norm)*sin(gyro_norm * dt * 0.25)*Omega)*q;

    }
    else{
        qt_dt = (Matrix4d::Identity()+0.5*dt*Omega) *
                cos(gyro_norm*dt*0.5) * q;
        qt_ddt = (Matrix4d::Identity()+0.25*dt*Omega) *
                 cos(gyro_norm*dt*0.25) * q;
        //qt_dt = (Matrix4d::Identity() - 0.5 * dt * Omega)*q;
        //qt_ddt = (Matrix4d::Identity() - 0.25 * dt * Omega)*q;
    }

    Vector3d k_v1,k_v2,k_v3,k_v4,k_p1,k_p2,k_p3,k_p4;

    k_v1 = quaternionToRotation(q).transpose() * acc + IMUState::gravity;
    k_p1 = v;

    k_v2 = quaternionToRotation(qt_ddt).transpose() * acc + IMUState::gravity;
    k_p2 = v + k_v1 * dt * 0.5;

    k_v3 = quaternionToRotation(qt_ddt).transpose() * acc + IMUState::gravity;
    k_p3 = v + k_v2 * dt * 0.5;

    k_v4 = quaternionToRotation(qt_dt).transpose() * acc + IMUState::gravity;
    k_p4 = v + k_v3 * dt;

    q = qt_dt;
    quaternionNormalize(q);
    v = v + (dt / 6) * (k_v1 + 2*k_v2 + 2*k_v3 + k_v4);
    p = p + (dt / 6) * (k_p1 + 2*k_p2 + 2*k_p3 + k_p4);

    return;
}

void propogate_model(const double &time, const Eigen::Vector3d &m_gyro, const Eigen::Vector3d &m_acc) {
    //首先，我们将误差值消除，得到真实角加速度和加速度
    IMUState& imu_state = state_server.imu_state;
    Vector3d gyro = m_gyro - imu_state.gyro_bias;
    Vector3d acc = m_acc - imu_state.acc_bias;
    double dtime = time - imu_state.time;  //这里的时间差就是IMU的开始时刻到当前图片帧的时间差，直接一个估算求出当前时刻的PVQ

    //基于此，计算出F,G,有 x' = F * x + G * n
    Matrix<double,21,21> F  = Matrix<double,21,21>::Zero();
    Matrix<double, 21, 12> G = Matrix<double, 21, 12>::Zero();

    F.block<3,3>(0,0) = -skewSymmetric(gyro);
    F.block<3,3>(0,3) = -Matrix3d::Identity();
    F.block<3,3>(6,0) = -quaternionToRotation(
            imu_state.orientation).transpose()*skewSymmetric(acc);
    F.block<3,3>(6,9) = quaternionToRotation(imu_state.orientation).transpose();
    F.block<3,3>(12,6) = Matrix3d::Identity();

    G.block<3,3>(0,0) = -Matrix3d::Identity();
    G.block<3,3>(3,3) = Matrix3d::Identity();
    G.block<3,3>(6,6) = -quaternionToRotation(imu_state.orientation).transpose();
    G.block<3,3>(9, 9) = Matrix3d::Identity();

    //利用四阶法，估算出当前时刻的PQV
    predictNewState(dtime,gyro,acc);

    //更新协方差
    Matrix<double, 21, 21> Fdt = F * dtime;
    Matrix<double, 21, 21> Fdt_square = Fdt * Fdt;
    Matrix<double, 21, 21> Fdt_cube = Fdt_square * Fdt;
    Matrix<double, 21, 21> Phi = Matrix<double, 21, 21>::Identity() +
                                 Fdt + 0.5*Fdt_square + (1.0/6.0)*Fdt_cube;

    //实际上，我们已经求出了phi，可以基于此进行协方差的优化了，但是这里做了进一步的迭代
    //原理还未找到，待询问
    // Modify the transition matrix
    Matrix3d R_kk_1 = quaternionToRotation(imu_state.orientation_null);
    Phi.block<3, 3>(0, 0) =
            quaternionToRotation(imu_state.orientation) * R_kk_1.transpose();

    Vector3d u = R_kk_1 * IMUState::gravity;
    RowVector3d s = (u.transpose()*u).inverse() * u.transpose();

    Matrix3d A1 = Phi.block<3, 3>(6, 0);
    Vector3d w1 = skewSymmetric(
            imu_state.velocity_null-imu_state.velocity) * IMUState::gravity;
    Phi.block<3, 3>(6, 0) = A1 - (A1*u-w1)*s;

    Matrix3d A2 = Phi.block<3, 3>(12, 0);
    Vector3d w2 = skewSymmetric(
            dtime*imu_state.velocity_null+imu_state.position_null-
            imu_state.position) * IMUState::gravity;
    Phi.block<3, 3>(12, 0) = A2 - (A2*u-w2)*s;
    //Phi update end here

    //预积分开始
    Matrix<double, 21, 21> Q = Phi*G*state_server.continuous_noise_cov*
                               G.transpose()*Phi.transpose()*dtime;
    state_server.state_cov.block<21,21>(0,0) =
            Phi * state_server.state_cov.block<21, 21>(0, 0)*Phi.transpose() + Q;

    if (state_server.cam_states.size() > 0) {
        state_server.state_cov.block(
                0, 21, 21, state_server.state_cov.cols()-21) =
                Phi * state_server.state_cov.block(
                        0, 21, 21, state_server.state_cov.cols()-21);
        state_server.state_cov.block(
                21, 0, state_server.state_cov.rows()-21, 21) =
                state_server.state_cov.block(
                        21, 0, state_server.state_cov.rows()-21, 21) * Phi.transpose();
    }

    MatrixXd state_cov_fixed = (state_server.state_cov +
                                state_server.state_cov.transpose()) / 2.0;
    state_server.state_cov = state_cov_fixed;

    // Update the state correspondes to null space.
    imu_state.orientation_null = imu_state.orientation;
    imu_state.position_null = imu_state.position;
    imu_state.velocity_null = imu_state.velocity;

    // Update the state info
    state_server.imu_state.time = time;
    return;
}

void propogate_process(const double &time) {
    int used_imu_num = 0;
    for(auto &imu_msg:Imu_Msg_Buffer){
        double imu_time = imu_msg.header.stamp.toSec();
        if(imu_time < state_server.imu_state.time){
            used_imu_num++;
            continue;
        }

        if(imu_time > time) break;

        //也就是说，image的那一帧的IMU信息来做预积分
        Eigen::Vector3d m_gyro, m_acc;
        tf::vectorMsgToEigen(imu_msg.angular_velocity, m_gyro);
        tf::vectorMsgToEigen(imu_msg.linear_acceleration, m_acc);

        //propogate process here
        propogate_model(imu_time,m_gyro,m_acc);
        used_imu_num++;


    }
    state_server.imu_state.id  = IMUState::next_id++;
    Imu_Msg_Buffer.erase(Imu_Msg_Buffer.begin(),Imu_Msg_Buffer.begin() + used_imu_num);
    return;
}

void addFeatureObservations(const sensor_msgs::PointCloudConstPtr &feature_msg_cloud) {
    StateIDType state_id = state_server.imu_state.id;
    int curr_feature_num = map_server.size();
    int tracked_feature_num = 0;

    //ids,life_time,u0,v0,u1,v1

    for(size_t i = 0;i<feature_msg_cloud->channels[0].values.size();i++){
        FeatureIDType feature_id = feature_msg_cloud->channels[0].values[i];
        double u0 = feature_msg_cloud->channels[2].values[i];
        double v0 = feature_msg_cloud->channels[3].values[i];
        double u1 = feature_msg_cloud->channels[4].values[i];
        double v1 = feature_msg_cloud->channels[5].values[i];;
        if(map_server.find(feature_id) == map_server.end()){

            map_server[feature_id] = Feature(feature_id);
            map_server[feature_id].observations[state_id] =
                    Vector4d(u0,v0,u1,v1);
        }
        else{
            map_server[feature_id].observations[state_id] =
                    Vector4d(u0,v0,u1,v1);
            tracked_feature_num++;
        }
    }
    tracking_rate =
            static_cast<double>(tracked_feature_num) /
            static_cast<double>(curr_feature_num);
    return;

}

void state_augmentation(const double &time) {
    const Matrix3d& R_i2c = state_server.imu_state.R_imu_cam0;
    const Vector3d& t_c2i = state_server.imu_state.t_cam0_imu;

    //add a new camera
    //直接用T[b->ck] = T[b->Ik]*T[I->C]就能得到R,t之间的关系

    //因为记录的旋转是w->ik的，所以这里要旋转。记住：旋转记录的是w->others，平移记录的是others->w
    Matrix3d R_ik2w = quaternionToRotation(state_server.imu_state.orientation).transpose();
    Matrix3d R_ck2w = R_ik2w * R_i2c;
    Vector3d t_ik2w = state_server.imu_state.position;
    Vector3d t_ck2w = R_ik2w * t_c2i + t_ik2w;

    state_server.cam_states[state_server.imu_state.id] = CAMState(state_server.imu_state.id);
    CAMState& cam_state = state_server.cam_states[
            state_server.imu_state.id];

    cam_state.time = time;
    //奇怪的是，记录的位姿确实是记录了(w->c的旋转和c->w的平移/c在w下的坐标)
    cam_state.orientation = rotationToQuaternion(R_ck2w.transpose());
    cam_state.position = t_ck2w;

    cam_state.orientation_null = cam_state.orientation;
    cam_state.position_null = cam_state.position;

    Matrix<double,6,21> J = Matrix<double,6,21>::Zero();
    J.block<3,3>(0,0) = R_i2c;
    J.block<3, 3>(0, 15) = Matrix3d::Identity();
    //J.block<3, 3>(3, 0) = skewSymmetric(R_ik2w*t_c2i);
    J.block<3, 3>(3, 0) = -R_ik2w*skewSymmetric(t_c2i);
    J.block<3, 3>(3, 12) = Matrix3d::Identity();
    J.block<3, 3>(3, 18) = Matrix3d::Identity();

    // Resize the state covariance matrix.
    size_t old_rows = state_server.state_cov.rows();
    size_t old_cols = state_server.state_cov.cols();
    state_server.state_cov.conservativeResize(old_rows+6, old_cols+6);

    const Matrix<double,21,21>& P11 =
            state_server.state_cov.block<21,21>(0,0);
    const MatrixXd& P12 =
            state_server.state_cov.block(0,21,21,old_cols-21);

    // Fill in the augmented state covariance.
    state_server.state_cov.block(old_rows, 0, 6, old_cols) << J*P11, J*P12;
    state_server.state_cov.block(0, old_cols, old_rows, 6) =
            state_server.state_cov.block(old_rows, 0, 6, old_cols).transpose();
    state_server.state_cov.block<6, 6>(old_rows, old_cols) =
            J * P11 * J.transpose();

    // Fix the covariance to be symmetric
    MatrixXd state_cov_fixed = (state_server.state_cov +
                                state_server.state_cov.transpose()) / 2.0;
    state_server.state_cov = state_cov_fixed;
    return;
}

void measurementUpdate(const MatrixXd &H, const VectorXd &r) {
    if(H.rows() == 0 || r.cols() == 0){
        return;
    }

    MatrixXd H_thin;
    VectorXd r_thin;
    if (H.rows() > H.cols()) {
        // Convert H to a sparse matrix.
        SparseMatrix<double> H_sparse = H.sparseView();

        // Perform QR decompostion on H_sparse.
        SPQR<SparseMatrix<double> > spqr_helper;
        spqr_helper.setSPQROrdering(1);
        spqr_helper.compute(H_sparse);

        MatrixXd H_temp;
        VectorXd r_temp;
        (spqr_helper.matrixQ().transpose() * H).evalTo(H_temp);
        (spqr_helper.matrixQ().transpose() * r).evalTo(r_temp);

        H_thin = H_temp.topRows(21+state_server.cam_states.size()*6);
        r_thin = r_temp.head(21+state_server.cam_states.size()*6);

        //HouseholderQR<MatrixXd> qr_helper(H);
        //MatrixXd Q = qr_helper.householderQ();
        //MatrixXd Q1 = Q.leftCols(21+state_server.cam_states.size()*6);

        //H_thin = Q1.transpose() * H;
        //r_thin = Q1.transpose() * r;
    } else {
        H_thin = H;
        r_thin = r;
    }

    //EKF upodate
    const MatrixXd P = state_server.state_cov;
    MatrixXd S = H_thin * P * H_thin.transpose() +
                 Feature::observation_noise * MatrixXd::Identity(H_thin.rows(), H_thin.rows());
    MatrixXd K_transpose = S.ldlt().solve(H_thin*P);
    MatrixXd K = K_transpose.transpose();

    // Compute the error of the state.
    VectorXd delta_x = K * r_thin;

    // Update the IMU state.
    const VectorXd& delta_x_imu = delta_x.head<21>();

    if (//delta_x_imu.segment<3>(0).norm() > 0.15 ||
        //delta_x_imu.segment<3>(3).norm() > 0.15 ||
            delta_x_imu.segment<3>(6).norm() > 0.5 ||
            //delta_x_imu.segment<3>(9).norm() > 0.5 ||
            delta_x_imu.segment<3>(12).norm() > 1.0) {
        printf("delta velocity: %f\n", delta_x_imu.segment<3>(6).norm());
        printf("delta position: %f\n", delta_x_imu.segment<3>(12).norm());
        ROS_WARN("Update change is too large.");
        //return;
    }

    const Vector4d dq_imu = smallAngleQuaternion(delta_x_imu.head<3>());
    state_server.imu_state.orientation = quaternionMultiplication(
            dq_imu, state_server.imu_state.orientation);
    state_server.imu_state.gyro_bias += delta_x_imu.segment<3>(3);
    state_server.imu_state.velocity += delta_x_imu.segment<3>(6);
    state_server.imu_state.acc_bias += delta_x_imu.segment<3>(9);
    state_server.imu_state.position += delta_x_imu.segment<3>(12);

    const Vector4d dq_extrinsic =
            smallAngleQuaternion(delta_x_imu.segment<3>(15));
    state_server.imu_state.R_imu_cam0 = quaternionToRotation(
            dq_extrinsic) * state_server.imu_state.R_imu_cam0;
    state_server.imu_state.t_cam0_imu += delta_x_imu.segment<3>(18);

    //update the camera states
    auto cam_iter = state_server.cam_states.begin();
    for(size_t i=0;i<state_server.cam_states.size();i++,cam_iter++){
        const VectorXd& delta_x_cam = delta_x.segment<6>(21 + i*6);
        const Vector4d dq_cam = smallAngleQuaternion(delta_x_cam.head<3>());
        cam_iter->second.orientation = quaternionMultiplication(dq_cam,cam_iter->second.orientation);
        cam_iter->second.position += delta_x_cam.tail<3>();
    }

    // Update state covariance.
    MatrixXd I_KH = MatrixXd::Identity(K.rows(), H_thin.cols()) - K*H_thin;
    //state_server.state_cov = I_KH*state_server.state_cov*I_KH.transpose() +
    //  K*K.transpose()*Feature::observation_noise;
    state_server.state_cov = I_KH*state_server.state_cov;

    // Fix the covariance to be symmetric
    MatrixXd state_cov_fixed = (state_server.state_cov +
                                state_server.state_cov.transpose()) / 2.0;
    state_server.state_cov = state_cov_fixed;
    return;
}

bool gatingTest(const MatrixXd &H, const VectorXd &r, const int &dof) {
    MatrixXd P1 = H * state_server.state_cov * H.transpose();
    MatrixXd P2 = Feature::observation_noise * MatrixXd::Identity(H.rows(),H.rows());
    double gamma = r.transpose() * (P1 + P2).ldlt().solve(r);

    if(gamma < chi_squared_test_table[dof]){
        return true;
    } else{
        return false;
    }
}

void measurementJacobian(const StateIDType &cam_state_id, const FeatureIDType &feature_id,
                         Eigen::Matrix<double, 4, 6> &H_x, Eigen::Matrix<double, 4, 3> &H_f,
                         Eigen::Vector4d &r) {
    const CAMState cam_state = state_server.cam_states[cam_state_id];
    const Feature feature_state = map_server[feature_id];

    //cam info
    Matrix3d R_w_c0 = quaternionToRotation(cam_state.orientation);
    Vector3d t_c0_w = cam_state.position;

    Matrix3d R_c0_c1 = CAMState::T_cam0_cam1.linear();
    Matrix3d R_w_c1 = R_c0_c1 * R_w_c0;
    Vector3d t_c1_w = t_c0_w - R_w_c1.transpose() * CAMState::T_cam0_cam1.translation();

    const Vector3d& p_w = feature_state.position;
    const Vector4d& z = feature_state.observations.find(cam_state_id)->second;

    Vector3d p_c0 = R_w_c0 * (p_w-t_c0_w);
    Vector3d p_c1 = R_w_c1 * (p_w-t_c1_w);

    Matrix<double,4,3> dz_dpc0 = Matrix<double,4,3>::Zero();
    dz_dpc0(0,0) = 1 / p_c0(2);
    dz_dpc0(1,1) = 1 / p_c0(2);
    dz_dpc0(0,2) = -p_c0(0) / (p_c0(2) * p_c0(2));
    dz_dpc0(1,2) = -p_c0(1) / (p_c0(2) * p_c0(2));

    Matrix<double,4,3> dz_dpc1 = Matrix<double,4,3>::Zero();
    dz_dpc1(0,0) = 1 / p_c1(2);
    dz_dpc1(1,1) = 1 / p_c1(2);
    dz_dpc1(0,2) = -p_c1(0) / (p_c1(2) * p_c1(2));
    dz_dpc1(1,2) = -p_c1(1) / (p_c1(2) * p_c1(2));

    Matrix<double,3,6> dpc0_dxc = Matrix<double,3,6>::Zero();
    dpc0_dxc.leftCols(3) = skewSymmetric(p_c0);
    dpc0_dxc.rightCols(3) = -R_w_c0;

    Matrix<double,3,6> dpc1_dxc = Matrix<double,3,6>::Zero();
    dpc1_dxc.leftCols(3) = skewSymmetric(p_c1);
    dpc1_dxc.rightCols(3) = -R_w_c1;

    Matrix3d dpc0_dpg = R_w_c0;
    Matrix3d dpc1_dpg = R_w_c1;

    // Modifty the measurement Jacobian to ensure
    // observability constrain.
    Matrix<double, 4, 6> A = H_x;
    Matrix<double, 6, 1> u = Matrix<double, 6, 1>::Zero();
    u.block<3, 1>(0, 0) = quaternionToRotation(
            cam_state.orientation_null) * IMUState::gravity;
    u.block<3, 1>(3, 0) = skewSymmetric(
            p_w-cam_state.position_null) * IMUState::gravity;
    H_x = A - A*u*(u.transpose()*u).inverse()*u.transpose();
    H_f = -H_x.block<4, 3>(0, 3);

    H_x = dz_dpc0*dpc0_dxc + dz_dpc1*dpc1_dxc;
    H_f = dz_dpc0*dpc0_dpg + dz_dpc1*dpc1_dpg;


    // Compute the residual.
    r = z - Vector4d(p_c0(0)/p_c0(2), p_c0(1)/p_c0(2),
                     p_c1(0)/p_c1(2), p_c1(1)/p_c1(2));
    return;
}

void featureJacobian(const FeatureIDType &feature_id, const std::vector<StateIDType> &cam_state_ids,
                     Eigen::MatrixXd &H_x, Eigen::VectorXd &r) {
    const auto& feature = map_server[feature_id];

    vector<StateIDType> valid_cam_ids;
    for(const auto& cam_id:cam_state_ids){
        if(feature.observations.find(cam_id) == feature.observations.end()) continue;
        valid_cam_ids.push_back(cam_id);
    }

    int jacobian_row_size = 0;
    jacobian_row_size = 4 * valid_cam_ids.size();
    MatrixXd H_xj = MatrixXd::Zero(jacobian_row_size,
                                   21 + state_server.cam_states.size()*6);
    MatrixXd H_fj = MatrixXd::Zero(jacobian_row_size,3);
    VectorXd r_j = VectorXd::Zero(jacobian_row_size);

    int jacobian_position = 0;
    for(const auto& cam_id : valid_cam_ids){
        Matrix<double,4,6> H_xi = Matrix<double,4,6>::Zero();
        Matrix<double,4,3> H_fi = Matrix<double,4,3>::Zero();
        Vector4d r_i = Vector4d::Zero();

        measurementJacobian(cam_id,feature_id,H_xi,H_fi,r_i);

        auto cam_state_iter = state_server.cam_states.find(cam_id);
        int cam_position = std::distance(state_server.cam_states.begin(),
                                         cam_state_iter);
        H_xj.block<4,6>(jacobian_position,21 + cam_position * 6) = H_xi;
        H_fj.block<4,3>(jacobian_position,0) = H_fi;
        r_j.segment<4>(jacobian_position) = r_i;
        jacobian_position +=4;
    }

    //Givens rotaions可以实现对矩阵特定位置的zero化，这里采用的是SVD来进行左零旋转的实现

    //这里svd分解是因为H_fj的秩为3，所以U的后n-3就是H_fj的左零空间
    /*
     *对于矩阵A,有四个空间：行空间、列空间、零空间、左零空间
     * 对于A(m*n),秩为r。进行SVD分解后有：U的前r列为A的列空间正交基，V的前r行为A的行空间正交基
     * U的后m-r列为A的左零空间正交基，V的后n-r行为A的零空间正交基
     * 因为A = U*S*VT  =>   A * V = U * S,对于S中的0项，就有A * V = 0
     * 也就是说Ａ的每一行都可以经过Ｖ的转换得到零空间中
    */
    JacobiSVD<MatrixXd> svd_helper(H_fj, ComputeFullU | ComputeThinV);
    MatrixXd A = svd_helper.matrixU().rightCols(
            jacobian_row_size - 3);

    H_x = A.transpose() * H_xj;
    r = A.transpose() * r_j;
    return;
}

void removeLostFeatures() {
    int jacobian_rows_num = 0;
    vector <FeatureIDType> invalid_feature_ids;
    vector <FeatureIDType> processed_feature_ids;

    for (auto iter = map_server.begin(); iter != map_server.end(); iter++) {
        auto &feature = iter->second;
        if (feature.observations.find(state_server.imu_state.id) !=
            feature.observations.end()) {
            continue;
        }
        if (feature.observations.size() < 3) {
            invalid_feature_ids.push_back(feature.id);
            continue;
        }

        if (!feature.is_initialized) {
            if (!feature.checkMotion(state_server.cam_states)) {
                invalid_feature_ids.push_back(feature.id);
                continue;
            } else {
                if (!feature.initializePosition(state_server.cam_states)) {
                    invalid_feature_ids.push_back(feature.id);
                    continue;
                }
            }
            jacobian_rows_num += 4*feature.observations.size() - 3;
            //cout << "feature_j id: " << feature.id << "  observations size: " << feature.observations.size() << endl;
            processed_feature_ids.push_back(feature.id);
        }
    }

    for(const auto& feature_id:invalid_feature_ids){
        map_server.erase(feature_id);
    }

    //cout << "jacobian all size " << jacobian_rows_num << endl;
    //cout << "process size "  << processed_feature_ids.size() << endl;
    if(processed_feature_ids.size() == 0){
        return;
    }
    //这是将所有的点的信息汇总到一起，并且进行了左零空间的变换之后得到的最终等式的size
    MatrixXd H_x = MatrixXd::Zero(jacobian_rows_num,
                                  21 + 6*state_server.cam_states.size());
    VectorXd r = VectorXd::Zero(jacobian_rows_num);

    int stack_num = 0;

    for(const auto& feature_id:processed_feature_ids){
        auto& feature = map_server[feature_id];

        vector<StateIDType> cam_ids;
        for(const auto& measurement:feature.observations){
            cam_ids.push_back(measurement.first);
        }

        MatrixXd H_xj;
        VectorXd r_j;

        featureJacobian(feature.id,cam_ids,H_xj,r_j);

        if(gatingTest(H_xj,r_j,cam_ids.size() - 1)){
            H_x.block(stack_num,0,H_xj.rows(),H_xj.cols()) = H_xj;
            r.segment(stack_num,r_j.rows()) = r_j;
            stack_num += H_xj.rows();
        }
        if (stack_num > 1500) break;
    }

    H_x.conservativeResize(stack_num, H_x.cols());
    r.conservativeResize(stack_num);

    measurementUpdate(H_x,r);
    //cout << "measurementupdate ok" << endl;
    // Remove all processed features from the map.
    for (const auto& feature_id : processed_feature_ids)
        map_server.erase(feature_id);
    return;
}

void findRemoveCameraids(std::vector<StateIDType>& rm_cam_ids) {
    auto key_cam_iter = state_server.cam_states.end();
    for(int i=0;i < 4;i++){
        key_cam_iter--;
    }
    auto choose_cam_iter = key_cam_iter;
    choose_cam_iter++;

    auto first_cam_iter = state_server.cam_states.begin();

    const Vector3d key_position = key_cam_iter->second.position;
    const Matrix3d key_rotation = quaternionToRotation(key_cam_iter->second.orientation);

    for(int i=0;i<2;i++){
        const Vector3d choose_position = choose_cam_iter->second.position;
        const Matrix3d choose_rotation = quaternionToRotation(choose_cam_iter->second.orientation);

        double distance = (key_position - choose_position).norm();
        double angle = AngleAxisd(key_rotation * choose_rotation.transpose()).angle();

        if(angle < rotation_threshold &&
           distance < translation_threshold &&
           tracking_rate > tracking_rate_threshold
                ){
            rm_cam_ids.push_back(choose_cam_iter->first);
            choose_cam_iter++;
        }   else{
            rm_cam_ids.push_back(first_cam_iter->first);
            first_cam_iter++;
        }
    }
    sort(rm_cam_ids.begin(),rm_cam_ids.end());
    return;
}

void pruneCamStateBuffer() {
    if(state_server.cam_states.size() < max_cam_state_size){
        return;
    }
    vector<StateIDType> rm_cam_state_ids;

    findRemoveCameraids(rm_cam_state_ids);

    int jacobian_row_size = 0;
    for(auto &item:map_server){
        auto& feature = item.second;
        vector<StateIDType> involved_cam_ids;
        for(const auto& cam_id:rm_cam_state_ids){
            if(feature.observations.find(cam_id) != feature.observations.end()){
                involved_cam_ids.push_back(cam_id);
            }
        }

        if(involved_cam_ids.size() == 0) continue;
        //一个太少了，无法进行cam_update
        if(involved_cam_ids.size() == 1){
            feature.observations.erase(involved_cam_ids[0]);
            continue;
        }

        if(!feature.is_initialized){
            if(!feature.checkMotion(state_server.cam_states)){
                for(auto cam_id : involved_cam_ids){
                    feature.observations.erase(cam_id);
                }
                continue;
            } else {
                if(!feature.initializePosition(state_server.cam_states)){
                    for (const auto& cam_id : involved_cam_ids)
                        feature.observations.erase(cam_id);
                    continue;
                }
            }
        }
        jacobian_row_size +=4 * involved_cam_ids.size() - 3;
    }

    // Compute the Jacobian and residual.
    MatrixXd H_x = MatrixXd::Zero(jacobian_row_size,
                                  21+6*state_server.cam_states.size());
    VectorXd r = VectorXd::Zero(jacobian_row_size);
    int stack_cntr = 0;

    for (auto& item : map_server) {
        auto& feature = item.second;
        // Check how many camera states to be removed are associated
        // with this feature.
        vector<StateIDType> involved_cam_state_ids(0);
        for (const auto& cam_id : rm_cam_state_ids) {
            if (feature.observations.find(cam_id) !=
                feature.observations.end())
                involved_cam_state_ids.push_back(cam_id);
        }

        if (involved_cam_state_ids.size() == 0) continue;

        MatrixXd H_xj;
        VectorXd r_j;
        featureJacobian(feature.id, involved_cam_state_ids, H_xj, r_j);

        if (gatingTest(H_xj, r_j, involved_cam_state_ids.size())) {
            H_x.block(stack_cntr, 0, H_xj.rows(), H_xj.cols()) = H_xj;
            r.segment(stack_cntr, r_j.rows()) = r_j;
            stack_cntr += H_xj.rows();
        }

        for (const auto& cam_id : involved_cam_state_ids)
            feature.observations.erase(cam_id);
    }

    H_x.conservativeResize(stack_cntr, H_x.cols());
    r.conservativeResize(stack_cntr);

    // Perform measurement update.
    measurementUpdate(H_x, r);

    for (const auto& cam_id : rm_cam_state_ids) {
        int cam_sequence = std::distance(state_server.cam_states.begin(),
                                         state_server.cam_states.find(cam_id));
        int cam_state_start = 21 + 6*cam_sequence;
        int cam_state_end = cam_state_start + 6;

        // Remove the corresponding rows and columns in the state
        // covariance matrix.
        if (cam_state_end < state_server.state_cov.rows()) {
            state_server.state_cov.block(cam_state_start, 0,
                                         state_server.state_cov.rows()-cam_state_end,
                                         state_server.state_cov.cols()) =
                    state_server.state_cov.block(cam_state_end, 0,
                                                 state_server.state_cov.rows()-cam_state_end,
                                                 state_server.state_cov.cols());

            state_server.state_cov.block(0, cam_state_start,
                                         state_server.state_cov.rows(),
                                         state_server.state_cov.cols()-cam_state_end) =
                    state_server.state_cov.block(0, cam_state_end,
                                                 state_server.state_cov.rows(),
                                                 state_server.state_cov.cols()-cam_state_end);

            state_server.state_cov.conservativeResize(
                    state_server.state_cov.rows()-6, state_server.state_cov.cols()-6);
        } else {
            state_server.state_cov.conservativeResize(
                    state_server.state_cov.rows()-6, state_server.state_cov.cols()-6);
        }

        // Remove this camera state in the state vector.
        state_server.cam_states.erase(cam_id);
    }


    return;
}

void publish(const ros::Time &time) {
    return;
}

void onlineReset() {
    if(position_std_threshold < 0){
        return;
    }
    static long long int online_reset_counter = 0;

    // Check the uncertainty of positions to determine if
    // the system can be reset.
    double position_x_std = std::sqrt(state_server.state_cov(12, 12));
    double position_y_std = std::sqrt(state_server.state_cov(13, 13));
    double position_z_std = std::sqrt(state_server.state_cov(14, 14));

    if (position_x_std < position_std_threshold &&
        position_y_std < position_std_threshold &&
        position_z_std < position_std_threshold) return;

    ROS_WARN("Start %lld online reset procedure...",
             ++online_reset_counter);
    ROS_INFO("Stardard deviation in xyz: %f, %f, %f",
             position_x_std, position_y_std, position_z_std);

    ROS_WARN("Start %lld online reset procedure...",
             ++online_reset_counter);
    ROS_INFO("Stardard deviation in xyz: %f, %f, %f",
             position_x_std, position_y_std, position_z_std);

    // Remove all existing camera states.
    state_server.cam_states.clear();

    // Clear all exsiting features in the map.
    map_server.clear();

    double gyro_bias_cov, acc_bias_cov, velocity_cov;
    gyro_bias_cov = 1e-4;
    velocity_cov = 0.25;
    acc_bias_cov = 1e-2;

    double extrinsic_rotation_cov, extrinsic_translation_cov;
    extrinsic_rotation_cov = 3.0462e-4;
    extrinsic_translation_cov = 1e-4;
    state_server.state_cov = MatrixXd::Zero(21, 21);
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

    ROS_WARN("%lld online reset complete...", online_reset_counter);
    return;
}

void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg){
    if(!is_gravity_set) return;
    if(is_first_img){
        is_first_img = 0;
        //记录下初始image的时间
        state_server.imu_state.time = feature_msg->header.stamp.toSec();
    }

    static double max_processing_time = 0.0;
    static int critical_time_cntr = 0;

    double processing_start_time = ros::Time::now().toSec();

    cout << '\n' << endl;
    cout << "iter start: " << iter_num++ << endl;
    cout << "feature info size " << feature_msg->header.stamp.toSec() << endl;
    ros::Time start_time = ros::Time::now();
    propogate_process(feature_msg->header.stamp.toSec());
    double propogate_process_time = (ros::Time::now() - start_time).toSec();
    cout << "propogate " << endl;

    start_time = ros::Time::now();
    state_augmentation(feature_msg->header.stamp.toSec());
    double state_augmentation_time = (ros::Time::now() - start_time).toSec();
    cout << "augmentation " << endl;

    start_time = ros::Time::now();
    addFeatureObservations(feature_msg);
    double add_observations_time = (
            ros::Time::now()-start_time).toSec();
    cout << "addfeature " << endl;

    start_time = ros::Time::now();
    removeLostFeatures();
    double remove_lost_features_time = (
            ros::Time::now()-start_time).toSec();
    cout << "remove " << endl;

    start_time = ros::Time::now();
    pruneCamStateBuffer();
    double prune_cam_states_time = (
            ros::Time::now()-start_time).toSec();
    cout << "prunecam " << endl;

    // Publish the odometry.
    start_time = ros::Time::now();
    publish(feature_msg->header.stamp);
    double publish_time = (
            ros::Time::now()-start_time).toSec();

    //onlineReset();

    return;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "track_node");

    ros::NodeHandle n("~");
    if(!initilization(n)){
        cout << "tracking failed when initializing !" << endl;
        return false;
    };
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);


    ros::Subscriber track_imu_sub = n.subscribe("/imu0",40,imu_callback);
    ros::Subscriber feature_sub = n.subscribe("/image_node/features",40,feature_callback);

    //pub_vio_path = n.advertise<nav_msgs::Path>("no_loop_path", 1000);
    //pub_key_odometrys = n.advertise<visualization_msgs::Marker>("key_odometrys", 1000);

    ros::spin();
    return 0;

}