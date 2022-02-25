/*
 * @Description: 
 * @Author: Ren Qian
 * @Date: 2020-02-28 19:17:00
 */
#include "lidar_localization/sensor_data/key_frame.hpp"

namespace lidar_localization {
<<<<<<< HEAD
Eigen::Quaternionf KeyFrame::GetQuaternion() {
=======

Eigen::Quaternionf KeyFrame::GetQuaternion() const {
>>>>>>> 124da3fd8c72597742a4c00e8aa60a9369b719e3
    Eigen::Quaternionf q;
    q = pose.block<3,3>(0,0);

    return q;
}
<<<<<<< HEAD
=======

Eigen::Vector3f KeyFrame::GetTranslation() const {
    Eigen::Vector3f t = pose.block<3,1>(0,3);

    return t;
}

>>>>>>> 124da3fd8c72597742a4c00e8aa60a9369b719e3
}