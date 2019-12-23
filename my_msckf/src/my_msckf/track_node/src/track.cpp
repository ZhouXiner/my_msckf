//
// Created by zhouxin on 2019/12/23.
//
#include "track.h"

MSCKF_VIO::MSCKF_VIO() {

}
bool MSCKF_VIO::initialization(){
    static_cam_state = CAMState();
    static_imu_state = IMUState();
    static_imu_state.next_id = 0;
    return true;
}

void MSCKF_VIO::imu_track(const sensor_msgs::ImuConstPtr& msg){
    MSCKF_VIO::Imu_Msg_Buffer.push_back(*msg);
}