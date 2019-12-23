//
// Created by zhouxin on 2019/12/9.
//
#include <image_node/utils.h>
#include <vector>

namespace utils {

    Eigen::Isometry3d getTransformEigen(const ros::NodeHandle &nh,
                                        const std::string &field) {
        Eigen::Isometry3d T;
        cv::Mat c = getTransformCV(nh, field);

        T.linear()(0, 0)   = c.at<double>(0, 0);
        T.linear()(0, 1)   = c.at<double>(0, 1);
        T.linear()(0, 2)   = c.at<double>(0, 2);
        T.linear()(1, 0)   = c.at<double>(1, 0);
        T.linear()(1, 1)   = c.at<double>(1, 1);
        T.linear()(1, 2)   = c.at<double>(1, 2);
        T.linear()(2, 0)   = c.at<double>(2, 0);
        T.linear()(2, 1)   = c.at<double>(2, 1);
        T.linear()(2, 2)   = c.at<double>(2, 2);
        T.translation()(0) = c.at<double>(0, 3);
        T.translation()(1) = c.at<double>(1, 3);
        T.translation()(2) = c.at<double>(2, 3);
        return T;
    }

    Eigen::Matrix4d getT(const vector<double> t_vec){
        Eigen::Matrix4d T;
        T << t_vec[0],t_vec[1],t_vec[2],t_vec[3],
             t_vec[4],t_vec[5],t_vec[6],t_vec[7],
             t_vec[8],t_vec[9],t_vec[10],t_vec[11],
             t_vec[12],t_vec[13],t_vec[14],t_vec[15];
        return T;
    }
}