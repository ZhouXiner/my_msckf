//
// Created by zhouxin on 2019/12/6.
//

#ifndef SRC_FEATURE_H
#define SRC_FEATURE_H
#include <iostream>
#include <vector>

using namespace std;
using namespace Eigen;



struct Feature_Status{
    long long int feat_id;
    int life_time;
    cv::Point2f p0;
    cv::Point2f p1;
};

struct Feature_Grid{
    cv::Point2f p0;
    cv::Point2f p1;
    float response;
};

#endif //SRC_FEATURE_H
