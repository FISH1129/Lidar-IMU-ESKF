<<<<<<< HEAD

=======
/*
 * @Description: synced GNSS-odo measurements as PosVelData
 * @Author: Ge Yao
 * @Date: 2020-11-21 15:39:24
 */
>>>>>>> 124da3fd8c72597742a4c00e8aa60a9369b719e3
#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_POS_VEL_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_POS_VEL_DATA_HPP_

#include <string>

#include <Eigen/Dense>

<<<<<<< HEAD
namespace lidar_localization{

class PosVelData{
    public:
        double time = 0.0;

        Eigen::Vector3f pos = Eigen::Vector3f::Zero();
        Eigen::Vector3f vel = Eigen::Vector3f::Zero();
=======
namespace lidar_localization {

class PosVelData {
  public:
    double time = 0.0;

    Eigen::Vector3f pos = Eigen::Vector3f::Zero();
    Eigen::Vector3f vel = Eigen::Vector3f::Zero();
>>>>>>> 124da3fd8c72597742a4c00e8aa60a9369b719e3
};

} // namespace lidar_localization

#endif // LIDAR_LOCALIZATION_SENSOR_DATA_POS_VEL_DATA_HPP_