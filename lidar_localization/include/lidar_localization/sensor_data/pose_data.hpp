/*
 * @Description: 存放处理后的IMU姿态以及GNSS位置
 * @Author: Ren Qian
 * @Date: 2020-02-27 23:10:56
 */
#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_POSE_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_POSE_DATA_HPP_

#include <Eigen/Dense>

namespace lidar_localization {
<<<<<<< HEAD
=======

>>>>>>> 124da3fd8c72597742a4c00e8aa60a9369b719e3
class PoseData {
  public:
    double time = 0.0;
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    Eigen::Vector3f vel = Eigen::Vector3f::Zero();
<<<<<<< HEAD
    
  public:
    Eigen::Quaternionf GetQuaternion();
};
=======

  public:
    Eigen::Quaternionf GetQuaternion();
};

>>>>>>> 124da3fd8c72597742a4c00e8aa60a9369b719e3
}

#endif