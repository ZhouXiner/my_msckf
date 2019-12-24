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

struct Feature{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef long long int FeatureIDType;

    // Optimization configuration for solving the 3d position.
    // Constructors for the struct.
    Feature(): id(0), position(Eigen::Vector3d::Zero()),
               is_initialized(false) {}

    Feature(const FeatureIDType& new_id): id(new_id),
                                          position(Eigen::Vector3d::Zero()),
                                          is_initialized(false) {}

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

};

typedef Feature::FeatureIDType FeatureIDType;
typedef std::map<FeatureIDType, Feature, std::less<int>,
        Eigen::aligned_allocator<
                std::pair<const FeatureIDType, Feature> > > MapServer;
#endif //SRC_FEATURES_H
