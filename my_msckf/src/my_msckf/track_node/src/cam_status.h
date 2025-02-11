 //
// Created by zhouxin on 2019/12/23.
//

#ifndef SRC_CAM_STATUS_H
#define SRC_CAM_STATUS_H
#include <map>
#include <vector>
#include <Eigen/Dense>

#include "imu_status.h"

struct  CAMState{
    StateIDType id;

    double time;

    // Orientation
    // Take a vector from the world frame to the camera frame.
    Eigen::Vector4d orientation;

    // Position of the camera frame in the world frame.
    Eigen::Vector3d position;

    // These two variables should have the same physical
    // interpretation with `orientation` and `position`.
    // There two variables are used to modify the measurement
    // Jacobian matrices to make the observability matrix
    // have proper null space.
    Eigen::Vector4d orientation_null;
    Eigen::Vector3d position_null;

    // Takes a vector from the cam0 frame to the cam1 frame.
    static Eigen::Isometry3d T_cam0_cam1;

    CAMState(): id(0), time(0),
                orientation(Eigen::Vector4d(0, 0, 0, 1)),
                position(Eigen::Vector3d::Zero()),
                orientation_null(Eigen::Vector4d(0, 0, 0, 1)),
                position_null(Eigen::Vector3d(0, 0, 0)) {}

    CAMState(const StateIDType& new_id ): id(new_id), time(0),
                orientation(Eigen::Vector4d(0, 0, 0, 1)),
                position(Eigen::Vector3d::Zero()),
                orientation_null(Eigen::Vector4d(0, 0, 0, 1)),
                position_null(Eigen::Vector3d::Zero()) {}
};

typedef std::map<StateIDType, CAMState, std::less<int>,
Eigen::aligned_allocator<
        std::pair<const StateIDType, CAMState> > > CamStateServer;
#endif //SRC_CAM_STATUS_H
