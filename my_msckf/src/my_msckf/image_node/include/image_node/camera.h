//
// Created by zhouxin on 2019/12/6.
//

#ifndef SRC_CAMERA_H
#define SRC_CAMERA_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>
#include <map>

using namespace std;
using namespace Eigen;

struct Camera_Status{
    long long int id;
    double time_stamp;
    Vector3d p_w_ci;
    Vector4d q_w_ci;
};

struct Camera_info{
};
#endif //SRC_CAMERA_H
