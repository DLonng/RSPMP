//#include "lidar_camera_fusion.h"
#include "lidar_camera_fusion.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "lidar_camera_fusion_node");

    LidarCameraFusion fusion_node;

    ros::spin();

    return 0;
}