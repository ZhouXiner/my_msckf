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
    double u0,u1,v0,v1;
};

#endif //SRC_FEATURE_H
