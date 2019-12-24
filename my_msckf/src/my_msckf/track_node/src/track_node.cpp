#include "track.h"

using namespace std;
using namespace Eigen;

MSCKF_VIO msckf_vio;

void imu_callback(const sensor_msgs::ImuConstPtr& msg){
    msckf_vio.imu_track(msg);
    return;
}

void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg){
    return;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "track_node");

    ros::NodeHandle n("~");
    if(!msckf_vio.initilization(n)){
        return false;
    };
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);


    ros::Subscriber track_imu_sub = n.subscribe("/imu0",40,imu_callback);
    ros::Subscriber feature_sub = n.subscribe("/image_node/features",40,feature_callback);
    
    ros::spin();
    return 0;

}