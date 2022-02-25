/*
 * @Description: 关键帧，在各个模块之间传递数据
 * @Author: Ren Qian
 * @Date: 2020-02-28 19:13:26
 */
#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_KEY_FRAME_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_KEY_FRAME_HPP_

#include <Eigen/Dense>

<<<<<<< HEAD
namespace lidar_localization
{
  class KeyFrame
  {
=======
namespace lidar_localization {
class KeyFrame {
>>>>>>> 124da3fd8c72597742a4c00e8aa60a9369b719e3
  public:
    double time = 0.0;
    unsigned int index = 0;
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

  public:
<<<<<<< HEAD
    Eigen::Quaternionf GetQuaternion();
  };
=======
    Eigen::Quaternionf GetQuaternion() const;
    Eigen::Vector3f GetTranslation() const;
};
>>>>>>> 124da3fd8c72597742a4c00e8aa60a9369b719e3
}

#endif