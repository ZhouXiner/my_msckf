#include <msckf/image_node.h>

bool initialize(ros::NodeHandle &nh){

    if(!load_parameter(nh)){
        cout << "bad parameter load " << endl;
        return false;
    }
    detector_ptr = cv::FastFeatureDetector::create(fast_threshold);
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
    publish();
}

void initialize_feature_track(){
    const cv::Mat& current_img0 = cam0_curr_img_ptr->image;
    const cv::Mat& current_img1 = cam1_curr_img_ptr->image;

    image_rows = current_img0.rows;
    image_cols = current_img0.cols;

    static int grid_height = current_img0.rows / grid_row;
    static int grid_width = current_img0.cols / grid_col;
    //grid_points_size = int(fast_threshold / (grid_col * grid_row));

    vector<cv::KeyPoint> cam0_KeyPoint(0);
    vector<cv::Point2f> cam0_points_undistorted;
    vector<cv::Point2f> cam1_points_undistorted;
    vector<cv::Point2f> cam0_inliers;
    vector<cv::Point2f> cam1_inliers;
    vector<cv::Point2f> cam0_points_before_track;
    vector<cv::Point2f> cam1_points_before_track;
    vector<vector<Feature_Grid>> Curr_Feature_Grid(grid_col * grid_row);
    Curr_Feature_Buffer.resize(grid_col * grid_row);

    detector_ptr->detect(current_img0,cam0_KeyPoint);


    cam0_points_before_track.resize(cam0_KeyPoint.size());
    for(size_t i=0;i<cam0_KeyPoint.size();i++){
        cam0_points_before_track[i] = cam0_KeyPoint[i].pt;
    }

    //得到了光流法左右追踪的结果，并且清除了效果不好的点
    //这里可以添加:先将cam0的点进行去畸变，得到归一化平面坐标，再利用两个平面间的R，t得到cam1预估值
    vector<uchar> status;
    vector<float> err;
    cv::calcOpticalFlowPyrLK(current_img0,current_img1,cam0_points_before_track,cam1_points_before_track,status,err,cv::Size(21, 21), 3);

    for(size_t i=0;i<status.size();i++){
        if(status[i] && inborder(cam1_points_before_track[i]) && inborder(cam0_points_before_track[i])){
            Feature_Grid feature_add;
            feature_add.p0 = cam0_points_before_track[i];
            feature_add.p1 = cam1_points_before_track[i];
            feature_add.response = cam0_KeyPoint[i].response;
            int row = int(cam0_points_before_track[i].x / grid_width);
            int col = int(cam0_points_before_track[i].y / grid_height);
            int id = row * grid_col + col;
            Curr_Feature_Grid[id].push_back(feature_add);

        }
    }


    for(auto &grid_feature:Curr_Feature_Grid){
        std::sort(grid_feature.begin(),grid_feature.end(),compare_by_response);
    }

    for(int id=0;id<Curr_Feature_Grid.size();id++){
        vector<Feature_Grid> grid_features = Curr_Feature_Grid[id];
        for(int size = 0;size < grid_points_min_size && size < grid_features.size();size++){
            Feature_Status feat;
            feat.feat_id = feature_id++;
            feat.life_time = 1;
            feat.p0 = grid_features[size].p0;
            feat.p1 = grid_features[size].p1;
            Curr_Feature_Buffer[id].push_back(feat);
        }
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

    static int grid_height = current_img0.rows / grid_row;
    static int grid_width = current_img0.cols / grid_col;
    Curr_Feature_Buffer.resize(grid_col * grid_row);

    vector<cv::Point2f> prev_cam0_points;
    vector<int> prev_cam0_lifetime;
    vector<long long int> prev_cam0_ids;


    for(auto prev_grid_features:Pre_Feature_Buffer){
        for(auto feat_info:prev_grid_features){
            prev_cam0_points.push_back(feat_info.p0);
            prev_cam0_ids.push_back(feat_info.feat_id);
            prev_cam0_lifetime.push_back(feat_info.life_time);
        }
    }

    int before_tracking = prev_cam0_points.size();

    //首先，利用Imu估计一个当前的点的分布

    vector<cv::Point2f> curr_cam0_track_points;
    vector<uchar> track_status;
    predict_track_points(curr_cam0_track_points,prev_cam0_points);

    cv::calcOpticalFlowPyrLK(prev_img,current_img0,prev_cam0_points,curr_cam0_track_points,track_status,cv::noArray());

    vector<cv::Point2f> prev_cam0_tracked_points;
    vector<int> prev_cam0_tracked_lifetime;
    vector<long long int> prev_cam0_tracked_ids;
    vector<cv::Point2f> curr_cam0_tracked_points;

    for(size_t i = 0;i<track_status.size();i++){
        if(!inborder(curr_cam0_track_points[i])){
            track_status[i] = 0;
        }
    }

    removeUnmarkedElements(prev_cam0_points,track_status,prev_cam0_tracked_points);
    removeUnmarkedElements(prev_cam0_ids,track_status,prev_cam0_tracked_ids);
    removeUnmarkedElements(prev_cam0_lifetime,track_status,prev_cam0_tracked_lifetime);
    removeUnmarkedElements(curr_cam0_track_points,track_status,curr_cam0_tracked_points);

    int after_tracking = curr_cam0_tracked_points.size();

   //光线追踪，确定左右相机的点
    vector<cv::Point2f> curr_cam1_match_points;
    vector<uchar> match_status;
    cv::calcOpticalFlowPyrLK(current_img0,current_img1,curr_cam0_tracked_points,curr_cam1_match_points,match_status,cv::noArray());

    vector<cv::Point2f> prev_cam0_matched_points;
    vector<int> prev_cam0_matched_lifetime;
    vector<long long int> prev_cam0_matched_ids;
    vector<cv::Point2f> curr_cam0_matched_points;
    vector<cv::Point2f> curr_cam1_matched_points;


    for(size_t i = 0;i<match_status.size();i++){
        if(!inborder(curr_cam1_match_points[i])){
            match_status[i] = 0;
        }
    }

    removeUnmarkedElements(prev_cam0_tracked_points,match_status,prev_cam0_matched_points);
    removeUnmarkedElements(prev_cam0_tracked_ids,match_status,prev_cam0_matched_ids);
    removeUnmarkedElements(prev_cam0_tracked_lifetime,match_status,prev_cam0_matched_lifetime);
    removeUnmarkedElements(curr_cam0_tracked_points,match_status,curr_cam0_matched_points);
    removeUnmarkedElements(curr_cam1_match_points,match_status,curr_cam1_matched_points);

    int after_matching = curr_cam1_matched_points.size();

    for(int i=0;i<curr_cam0_matched_points.size();i++){
        int row = int(curr_cam0_matched_points[i].y / grid_height);
        int col = int(curr_cam0_matched_points[i].x / grid_width);
        int code = row*grid_col + col;
        if(Curr_Feature_Buffer[code].size() > grid_points_min_size){
            continue;
        }
        Feature_Status feat;
        feat.feat_id = prev_cam0_matched_ids[i];
        feat.life_time = prev_cam0_matched_lifetime[i] + 1;
        feat.p0 = curr_cam0_matched_points[i];
        feat.p1 = curr_cam1_matched_points[i];
        Curr_Feature_Buffer[code].push_back(feat);
    }
    cout << "before tracking : " << before_tracking << " tracking: " << after_tracking << " matching: " << after_matching << endl;
    return;
}

void add_new_feature(){

    //生成mask
    const cv::Mat current_img0 = cam0_curr_img_ptr->image;
    const cv::Mat current_img1 = cam1_curr_img_ptr->image;

    static int grid_height = current_img0.rows / grid_row;
    static int grid_width = current_img0.cols / grid_col;


    cv::Mat mask(image_rows, image_cols, CV_8U, cv::Scalar(1));

    for(auto grid_features:Curr_Feature_Buffer){
        for(auto feat:grid_features) {
            const int y = feat.p0.x;
            const int x = feat.p0.y;

            int up_lim = y - 2, bottom_lim = y + 3,
                    left_lim = x - 2, right_lim = x + 3;
            if (up_lim < 0) up_lim = 0;
            if (bottom_lim > image_rows) bottom_lim = image_rows;
            if (left_lim < 0) left_lim = 0;
            if (right_lim > image_cols) right_lim = image_cols;

            cv::Range row_range(up_lim, bottom_lim);
            cv::Range col_range(left_lim, right_lim);
            mask(row_range, col_range) = 0;
        }
    }

    vector<cv::KeyPoint> cam0_KeyPoint;
    vector<cv::Point2f> cam0_points_undistorted;
    vector<cv::Point2f> cam1_points_undistorted;
    vector<cv::Point2f> cam0_inliers;
    vector<cv::Point2f> cam1_inliers;
    vector<cv::Point2f> cam0_points_before_track;
    vector<cv::Point2f> cam1_points_before_track;
    vector<vector<Feature_Grid>> Curr_Feature_Grid(grid_col * grid_row);

    detector_ptr->detect(current_img0,cam0_KeyPoint,mask);

    for(auto keypoint:cam0_KeyPoint){
        cam0_points_before_track.push_back(keypoint.pt);
    }

    //得到了光流法左右追踪的结果，并且清除了效果不好的点
    //这里可以添加:先将cam0的点进行去畸变，得到归一化平面坐标，再利用两个平面间的R，t得到cam1预估值
    vector<uchar> status;
    vector<float> err;
    cv::calcOpticalFlowPyrLK(current_img0,current_img1,cam0_points_before_track,cam1_points_before_track,status,err,cv::Size(21, 21), 3);

    for(size_t i=0;i<status.size();i++){
        if(status[i] && inborder(cam0_points_before_track[i]) && inborder(cam1_points_before_track[i])){
            Feature_Grid feature_add;
            feature_add.p0 = cam0_points_before_track[i];
            feature_add.p1 = cam1_points_before_track[i];
            feature_add.response = cam0_KeyPoint[i].response;
            int row = int(cam0_points_before_track[i].x / grid_width);
            int col = int(cam0_points_before_track[i].y / grid_height);
            int id = row * grid_col + col;
            Curr_Feature_Grid[id].push_back(feature_add);
        }
    }

    for(auto &grid_feature:Curr_Feature_Grid){
        std::sort(grid_feature.begin(),grid_feature.end(),compare_by_response);
    }

    cout << "grid finished !" << endl;
    int new_add = 0;
    for(int id=0;id<Curr_Feature_Grid.size();id++){
        vector<Feature_Grid> grid_features = Curr_Feature_Grid[id];
        for(int size = 0;size < grid_points_max_size && size < grid_features.size();size++){
            if(Curr_Feature_Buffer[id].size() >= grid_points_max_size){
                break;
            }

            Feature_Status feat;
            feat.feat_id = feature_id++;
            feat.life_time = 1;
            feat.p0 = grid_features[size].p0;
            feat.p1 = grid_features[size].p1;
            Curr_Feature_Buffer[id].push_back(feat);
            new_add++;
        }
    }
    cout << " new add: " << new_add << endl;
}

void predict_track_points(vector<cv::Point2f> &pt_predict,vector<cv::Point2f> pt_prev){
    if(pt_prev.size() == 0) return;

    pt_predict.resize(pt_prev.size());
    Matrix3d R_theta_c;
    predict_theta_R(R_theta_c);

    Matrix3d K;
    K << cam0_intrinsics[0], 0.0, cam0_intrinsics[2],
            0.0, cam0_intrinsics[1], cam0_intrinsics[3],
            0.0, 0.0, 1.0;
    Matrix3d H = K * R_theta_c * K.inverse();


    for(size_t i=0;i<pt_prev.size();i++){
        Vector3d p1(pt_prev[i].x,pt_prev[i].y,1.0);
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
    Pre_Feature_Buffer.clear();
    Pre_Feature_Buffer = Curr_Feature_Buffer;
    Curr_Feature_Buffer.clear();
    cam0_prev_img_ptr = cam0_curr_img_ptr;

}

bool inborder(cv::Point2f p){
    if(p.x < 0 || p.x > image_rows || p.y < 0 || p.y > image_cols){
        return false;
    }
    return true;
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
