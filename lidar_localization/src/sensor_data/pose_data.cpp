/*
 * @Description: 
 * @Author: Ren Qian
 * @Date: 2020-02-28 18:50:16
 */
#include "lidar_localization/sensor_data/pose_data.hpp"

namespace lidar_localization {
<<<<<<< HEAD
Eigen::Quaternionf PoseData::GetQuaternion() {
    Eigen::Quaternionf q;
    q = pose.block<3,3>(0,0);

    return q;
}
=======

Eigen::Quaternionf PoseData::GetQuaternion() {
    Eigen::Quaternionf q;
    q = pose.block<3,3>(0,0);
    return q;
}

>>>>>>> 124da3fd8c72597742a4c00e8aa60a9369b719e3
}