//
// Created by zhouxin on 2019/12/20.
//


void initializeGravityAndBias(){
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
    Imu_State.gyro_bias = sum_angular_vel / Imu_Msg_Buffer.size();

    Vector3d gravity_imu = sum_linear_acc / Imu_Msg_Buffer.size();

    double gravity_norm = gravity_imu.norm();

    Imu_State.gravity = Vector3d(0.0,0.0,gravity_norm);

    Quaterniond q0_i_w = Quaterniond::FromTwoVectors(gravity_imu,Imu_State.gravity);

    Imu_State.orientation = rotationToQuaternion(q0_i_w.toRotationMatrix().transpose());

    return;
}