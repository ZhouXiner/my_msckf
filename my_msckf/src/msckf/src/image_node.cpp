#include <msckf/image_node.h>

bool initialize(ros::NodeHandle &nh){

    if(!load_parameter(nh)){
        return false;
    }
    return true;

}

bool load_parameter(ros::NodeHandle &nh){
    // Camera calibration parameters
    nh.param<string>("cam0/distortion_model",
                     cam0_distortion_model, string("radtan"));
    nh.param<string>("cam1/distortion_model",
                     cam1_distortion_model, string("radtan"));

    vector<int> cam0_resolution_temp(2);
    nh.getParam("cam0/resolution", cam0_resolution_temp);
    cam0_resolution[0] = cam0_resolution_temp[0];
    cam0_resolution[1] = cam0_resolution_temp[1];

    vector<int> cam1_resolution_temp(2);
    nh.getParam("cam1/resolution", cam1_resolution_temp);
    cam1_resolution[0] = cam1_resolution_temp[0];
    cam1_resolution[1] = cam1_resolution_temp[1];

    vector<double> cam0_intrinsics_temp(4);
    nh.getParam("cam0/intrinsics", cam0_intrinsics_temp);
    cam0_intrinsics[0] = cam0_intrinsics_temp[0];
    cam0_intrinsics[1] = cam0_intrinsics_temp[1];
    cam0_intrinsics[2] = cam0_intrinsics_temp[2];
    cam0_intrinsics[3] = cam0_intrinsics_temp[3];

    vector<double> cam1_intrinsics_temp(4);
    nh.getParam("cam1/intrinsics", cam1_intrinsics_temp);
    cam1_intrinsics[0] = cam1_intrinsics_temp[0];
    cam1_intrinsics[1] = cam1_intrinsics_temp[1];
    cam1_intrinsics[2] = cam1_intrinsics_temp[2];
    cam1_intrinsics[3] = cam1_intrinsics_temp[3];

    vector<double> cam0_distortion_coeffs_temp(4);
    nh.getParam("cam0/distortion_coeffs",
                cam0_distortion_coeffs_temp);
    cam0_distortion_coeffs[0] = cam0_distortion_coeffs_temp[0];
    cam0_distortion_coeffs[1] = cam0_distortion_coeffs_temp[1];
    cam0_distortion_coeffs[2] = cam0_distortion_coeffs_temp[2];
    cam0_distortion_coeffs[3] = cam0_distortion_coeffs_temp[3];

    vector<double> cam1_distortion_coeffs_temp(4);
    nh.getParam("cam1/distortion_coeffs",
                cam1_distortion_coeffs_temp);
    cam1_distortion_coeffs[0] = cam1_distortion_coeffs_temp[0];
    cam1_distortion_coeffs[1] = cam1_distortion_coeffs_temp[1];
    cam1_distortion_coeffs[2] = cam1_distortion_coeffs_temp[2];
    cam1_distortion_coeffs[3] = cam1_distortion_coeffs_temp[3];

    vector<double> T_imu_cam0_temp(16);
    nh.getParam("cam0/T_cam_imu",T_imu_cam0_temp);
    Matrix4d T_imu_cam0 = getT(T_imu_cam0_temp);

    Matrix3d R_imu_cam0 = T_imu_cam0.block(0,0,3,3);
    Vector3d t_imu_cam0 = T_imu_cam0.block(0,3,3,1);
    /*
    P[imu] = R[imu,cam] * P[cam] + P[imu,cam]
    R[imu,cam]-1 * P[imu] - R[imu,cam]-1 * P[imu,cam] = P[cam]
    所以新的R,t可得
    */
    R_cam0_imu = R_imu_cam0.transpose();
    t_cam0_imu = -R_imu_cam0.transpose() * t_imu_cam0;

    vector<double> T_imu_cam1_temp(16);
    nh.getParam("cam1/T_cam_imu",T_imu_cam1_temp);
    Matrix4d T_imu_cam1 = getT(T_imu_cam1_temp);

    Matrix3d R_imu_cam1 = T_imu_cam1.block(0,0,3,3);
    Vector3d t_imu_cam1 = T_imu_cam1.block(0,3,3,1);
    /*
    P[imu] = R[imu,cam] * P[cam] + P[imu,cam]
    R[imu,cam]-1 * P[imu] - R[imu,cam]-1 * P[imu,cam] = P[cam]
    所以新的R,t可得
    */
    R_cam1_imu = R_imu_cam1.transpose();
    t_cam1_imu = -R_imu_cam1.transpose() * t_imu_cam1;

    ROS_INFO("===========================================");
    ROS_INFO("cam0_resolution: %d, %d",
             cam0_resolution[0], cam0_resolution[1]);
    ROS_INFO("cam0_intrinscs: %f, %f, %f, %f",
             cam0_intrinsics[0], cam0_intrinsics[1],
             cam0_intrinsics[2], cam0_intrinsics[3]);
    ROS_INFO("cam0_distortion_model: %s",
             cam0_distortion_model.c_str());
    ROS_INFO("cam0_distortion_coefficients: %f, %f, %f, %f",
             cam0_distortion_coeffs[0], cam0_distortion_coeffs[1],
             cam0_distortion_coeffs[2], cam0_distortion_coeffs[3]);

    ROS_INFO("cam1_resolution: %d, %d",
             cam1_resolution[0], cam1_resolution[1]);
    ROS_INFO("cam1_intrinscs: %f, %f, %f, %f",
             cam1_intrinsics[0], cam1_intrinsics[1],
             cam1_intrinsics[2], cam1_intrinsics[3]);
    ROS_INFO("cam1_distortion_model: %s",
             cam1_distortion_model.c_str());
    ROS_INFO("cam1_distortion_coefficients: %f, %f, %f, %f",
             cam1_distortion_coeffs[0], cam1_distortion_coeffs[1],
             cam1_distortion_coeffs[2], cam1_distortion_coeffs[3]);

    cout << "R_imu_cam0 " << R_imu_cam0 << endl;
    cout << "t_imu_cam0 " << t_imu_cam0.transpose() << endl;

    return true;
}

void imu_callback(const sensor_msgs::ImuConstPtr& msg){
    Imu_Msg_Buffer.push_back(* msg);
    return;
}



void stereo_callback(const sensor_msgs::ImageConstPtr& cam0_msg,
                     const sensor_msgs::ImageConstPtr& cam1_msg){

    cam0_curr_img_ptr = cv_bridge::toCvShare(cam0_msg,
                                             sensor_msgs::image_encodings::MONO8);
    cam1_curr_img_ptr = cv_bridge::toCvShare(cam1_msg,
                                             sensor_msgs::image_encodings::MONO8);

    if(!first_image_in){
        initialize_feature_track();
        first_image_in = 1;
    }
    else{
        feature_track();
        add_new_feature();
    }
    status_change();
}

void initialize_feature_track(){
    const cv::Mat current_img0 = cam0_curr_img_ptr->image;
    const cv::Mat current_img1 = cam1_curr_img_ptr->image;

    vector<cv::Point2f> cam0_points_undistorted;
    vector<cv::Point2f> cam1_points_undistorted;
    vector<cv::Point2f> cam0_points_tmp;
    vector<cv::Point2f> cam1_points_tmp;
    cv::goodFeaturesToTrack(current_img0, cam0_points_tmp,fast_threshold, 0.01,30);

    //得到了光流法左右追踪的结果，并且清除了效果不好的点
    vector<uchar> status;
    vector<float> err;
    cv::calcOpticalFlowPyrLK(current_img0,current_img1,cam0_points_tmp,cam1_points_tmp,status,err,cv::Size(21, 21), 3);

    for(size_t i=0;i<status.size();i++){
        if(status[i]){
            curr_cam0_points.push_back(cam0_points_tmp[i]);
            curr_cam1_points.push_back(cam1_points_tmp[i]);
        }
    }



    //应该对点进行undistort了，首先我们需要初始化内参、畸变信息等等
    undistort_points(curr_cam0_points,cam0_points_undistorted,cam0_intrinsics,cam0_distortion_coeffs);
    undistort_points(curr_cam1_points,cam1_points_undistorted,cam1_intrinsics,cam1_distortion_coeffs);

    //这里我们有很多可以增加的优化
    //1.利用本质矩阵，采用ransac筛选  2.对于点不够的情况，重新detect再进行匹配和去畸变  3.two_ransac等ransac的加强版
    for(size_t i=0;i<cam0_points_undistorted.size();i++){
        Feature_Status feat;
        feat.feat_id = feature_id++;
        feat.life_time = 1;
        feat.u0 = double(cam0_points_undistorted[i].x);
        feat.v0 = double(cam0_points_undistorted[i].y);
        feat.u1 = double(cam1_points_undistorted[i].x);
        feat.v1 = double(cam1_points_undistorted[i].y);
        Curr_Feature_Buffer.push_back(feat);
    }
}

void undistort_points(const vector<cv::Point2f>& pts_in,vector<cv::Point2f>& pts_out,
                      const Vector4d &intrinsics,const Vector4d &distortion_coeffs){
    cv::Vec4d cv_distortion_coeffs;

    cv::eigen2cv(distortion_coeffs,cv_distortion_coeffs);
    const cv::Matx33d K(
            intrinsics[0], 0.0, intrinsics[2],
            0.0, intrinsics[1], intrinsics[3],
            0.0, 0.0, 1.0);

    cv::undistortPoints(pts_in,pts_out,K,cv_distortion_coeffs,cv::noArray(),cv::noArray());

}


void feature_track(){
    const cv::Mat prev_img = cam0_prev_img_ptr->image;
    const cv::Mat current_img0 = cam0_curr_img_ptr->image;
    const cv::Mat current_img1 = cam1_curr_img_ptr->image;
    vector<uchar> track_status;
    vector<float> track_error;
    vector<uchar> stereo_status;
    vector<float> stereo_error;
    vector<cv::Point2f> current_points_tmp0;
    vector<cv::Point2f> current_points_tmp00;
    vector<cv::Point2f> current_points_undistorted0;
    vector<cv::Point2f> current_points_tmp1;
    vector<cv::Point2f> current_points_undistorted1;


    const int cols = current_img0.cols;
    const int rows = current_img0.rows;
    int track_cnt = 0;
    int stereo_cnt = 0;

    //首先，利用Imu估计一个当前的点的分布
    predict_track_points(current_points_tmp0);

    cout << "Prev: " << prev_cam0_points.size() << " before track： " << current_points_tmp0.size();

    //再进行光流追踪
    cv::calcOpticalFlowPyrLK(prev_img,current_img0,prev_cam0_points,current_points_tmp0,track_status,track_error);

    for(size_t i=0;i<track_status.size();i++){
        if(track_status[i] && (current_points_tmp0[i].x > 0 && current_points_tmp0[i].x < rows)
                  && (current_points_tmp0[i].y > 0 && current_points_tmp0[i].y < cols)){
            current_points_tmp00.push_back(current_points_tmp0[i]);
            track_id.push_back(i);
            track_cnt++;
        }
    }

    cv::calcOpticalFlowPyrLK(current_img0,current_img1,current_points_tmp00,current_points_tmp1,stereo_status,stereo_error);

    for(size_t i=0;i<stereo_status.size();i++){
        if(stereo_status[i] && track_status[track_id[i]] && (current_points_tmp1[i].x > 0 && current_points_tmp1[i].x < rows)
                                && (current_points_tmp1[i].y > 0 && current_points_tmp1[i].y < cols)){
            curr_cam0_points.push_back(current_points_tmp00[i]);
            curr_cam1_points.push_back(current_points_tmp1[i]);
        }
        else{
            track_id[i] = -1;
        }
    }

    //这里去畸变的点没有进行筛选，而是将所有点都放进去了，利用track_id进行判断
    undistort_points(current_points_tmp00,current_points_undistorted0,cam0_intrinsics,cam0_distortion_coeffs);
    undistort_points(current_points_tmp1,current_points_undistorted1,cam1_intrinsics,cam1_distortion_coeffs);


    for(size_t i=0;i<stereo_status.size();i++){
        if(stereo_status[i] && track_id[i] != -1 && track_status[track_id[i]] && (current_points_tmp1[i].x > 0 && current_points_tmp1[i].x < rows)
           && (current_points_tmp1[i].y > 0 && current_points_tmp1[i].y < cols)){
            int pre_index = track_id[i];
            Feature_Status feat;
            feat.feat_id = Pre_Feature_Buffer[pre_index].feat_id;
            feat.life_time = Pre_Feature_Buffer[pre_index].life_time + 1;
            feat.u0 = double(current_points_undistorted0[i].x);
            feat.v0 = double(current_points_undistorted0[i].y);
            feat.u1 = double(current_points_undistorted1[i].x);
            feat.v1 = double(current_points_undistorted1[i].y);
            Curr_Feature_Buffer.push_back(feat);
            stereo_cnt++;
        }
    }
    cout << " after track: " << curr_cam0_points.size() << ". Stereo size: " << stereo_cnt;

}

void add_new_feature(){

    if(curr_cam0_points.size() == fast_threshold) return;
    //生成mask
    const cv::Mat current_img0 = cam0_curr_img_ptr->image;
    const cv::Mat current_img1 = cam1_curr_img_ptr->image;
    const int cols = current_img0.cols;
    const int rows = current_img0.rows;

    cv::Mat mask(rows, cols, CV_8U, cv::Scalar(1));
    for(auto &feat:curr_cam0_points){
        const int y = feat.x;
        const int x = feat.y;

        int up_lim = y-2, bottom_lim = y+3,
                left_lim = x-2, right_lim = x+3;
        if (up_lim < 0) up_lim = 0;
        if (bottom_lim > rows) bottom_lim = rows;
        if (left_lim < 0) left_lim = 0;
        if (right_lim > cols) right_lim = cols;

        cv::Range row_range(up_lim, bottom_lim);
        cv::Range col_range(left_lim, right_lim);
        mask(row_range, col_range) = 0;
    }

    vector<cv::KeyPoint> add_keypoints;
    vector<cv::Point2f> add_cam0_points_tmp;
    vector<cv::Point2f> add_cam0_points_tmp0;
    vector<cv::Point2f> add_cam0_points_undistorted;
    vector<cv::Point2f> add_cam1_points_tmp;
    vector<cv::Point2f> add_cam1_points_tmp1;
    vector<cv::Point2f> add_cam1_points_undistorted;
    cv::goodFeaturesToTrack(current_img0, add_cam0_points_tmp,fast_threshold - curr_cam0_points.size(), 0.01,30,mask);
    for(auto feat:add_keypoints){
        add_cam0_points_tmp.push_back(feat.pt);
    }

    vector<unsigned char> add_status;
    cv::calcOpticalFlowPyrLK(current_img0,current_img1,add_cam0_points_tmp,add_cam1_points_tmp,add_status,cv::noArray());

    for(size_t i=0;i<add_status.size();i++){
        if(add_status[i] && (add_cam1_points_tmp[i].x > 0 && add_cam1_points_tmp[i].x < rows)
                            && (add_cam1_points_tmp[i].y > 0 && add_cam1_points_tmp[i].y < cols)){
            add_cam0_points_tmp0.push_back(add_cam0_points_tmp[i]);
            add_cam1_points_tmp1.push_back(add_cam1_points_tmp[i]);
        }
    }


    undistort_points(add_cam0_points_tmp0,add_cam0_points_undistorted,cam0_intrinsics,cam0_distortion_coeffs);
    undistort_points(add_cam1_points_tmp1,add_cam1_points_undistorted,cam1_intrinsics,cam1_distortion_coeffs);

    for(size_t i=0;i<add_cam0_points_undistorted.size();i++){
        Feature_Status feat;
        feat.feat_id = feature_id++;
        feat.life_time = 1;
        feat.u0 = double(add_cam0_points_undistorted[i].x);
        feat.v0 = double(add_cam0_points_undistorted[i].y);
        feat.u1 = double(add_cam1_points_undistorted[i].x);
        feat.v1 = double(add_cam1_points_undistorted[i].y);
        Curr_Feature_Buffer.push_back(feat);
    }
    curr_cam0_points.insert(curr_cam0_points.end(),add_cam0_points_tmp0.begin(),add_cam0_points_tmp0.end());

    cout << ". Add size: " << add_cam0_points_tmp0.size()  << endl;
}

void predict_track_points(vector<cv::Point2f> &pt_predict){
    if(prev_cam0_points.size() == 0) return;

    pt_predict.resize(prev_cam0_points.size());
    Matrix3d R_theta_c;
    predict_theta_R(R_theta_c);

    Matrix3d K;
    K << cam0_intrinsics[0], 0.0, cam0_intrinsics[2],
            0.0, cam0_intrinsics[1], cam0_intrinsics[3],
            0.0, 0.0, 1.0;
    Matrix3d H = K * R_theta_c * K.inverse();


    for(size_t i=0;i<prev_cam0_points.size();i++){
        Vector3d p1(prev_cam0_points[i].x,prev_cam0_points[i].y,1.0);
        Vector3d p2 = H * p1;
        pt_predict[i].x = p2[0] / p2[2];
        pt_predict[i].y = p2[1] / p2[2];
    }
    return;
}

void predict_theta_R(Matrix3d &R_theta_c){
    auto begin_iter = Imu_Msg_Buffer.begin();
    while (begin_iter != Imu_Msg_Buffer.end()) {
        if ((begin_iter->header.stamp-
             cam0_prev_img_ptr->header.stamp).toSec() < -0.01)
            ++begin_iter;
        else
            break;
    }

    auto end_iter = begin_iter;
    while (end_iter != Imu_Msg_Buffer.end()) {
        if ((end_iter->header.stamp-
             cam0_curr_img_ptr->header.stamp).toSec() < 0.005)
            ++end_iter;
        else
            break;
    }
    Vector3d mean_ang_vel(0.0, 0.0, 0.0);
    for (auto iter = begin_iter; iter < end_iter; ++iter)
        mean_ang_vel += Vector3d(iter->angular_velocity.x,
                              iter->angular_velocity.y, iter->angular_velocity.z);

    if (end_iter-begin_iter > 0)
        mean_ang_vel *= 1.0f / (end_iter-begin_iter);

    // Transform the mean angular velocity from the IMU
    // frame to the cam0 and cam1 frames.
    Vector3d cam0_mean_ang_vel = R_cam0_imu.transpose() * mean_ang_vel;

    // Compute the relative rotation.
    double dtime = (cam0_curr_img_ptr->header.stamp-
                    cam0_prev_img_ptr->header.stamp).toSec();

    //我们计算出了相机之间的平均角速度，从而求出了旋转向量，进而转换成旋转矩阵
    Vector3d theta_angle = cam0_mean_ang_vel*dtime;
    cv::Vec3d theta_angle_cv(theta_angle[0],theta_angle[1],theta_angle[2]);
    cv::Mat cam0_R_p_c;
    cv::Rodrigues(theta_angle_cv, cam0_R_p_c);
    cam0_R_p_c = cam0_R_p_c.t();
    cv::cv2eigen(cam0_R_p_c,R_theta_c);
    // Delete the useless and used imu messages.
    Imu_Msg_Buffer.erase(Imu_Msg_Buffer.begin(), end_iter);
    return;
}
void status_change(){
    Pre_Feature_Buffer = Curr_Feature_Buffer;
    Curr_Feature_Buffer.clear();
    prev_cam0_points.clear();
    cam0_prev_img_ptr = cam0_curr_img_ptr;
    prev_cam0_points = curr_cam0_points;
    prev_cam1_points = curr_cam1_points;
    curr_cam0_points.clear();
    curr_cam1_points.clear();
}


int main(int argc, char **argv){
    ros::init(argc, argv, "image_node");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    if(!initialize(n)) return false;

    message_filters::Subscriber<sensor_msgs::Image> cam0_img_sub(n,"/cam0/image_raw", 100);
    message_filters::Subscriber<sensor_msgs::Image> cam1_img_sub(n,"/cam1/image_raw", 100);
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> stereo_sub(cam0_img_sub, cam1_img_sub,100);
    stereo_sub.registerCallback(boost::bind(stereo_callback,_1,_2));

    ros::Subscriber imu_sub = n.subscribe("/imu0",100,imu_callback);

    ros::spin();
    return 0;
}
