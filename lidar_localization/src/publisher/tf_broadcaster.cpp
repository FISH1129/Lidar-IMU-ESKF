/*
 * @Description: 发布tf的类
 * @Author: Ren Qian
 * @Date: 2020-03-05 15:23:26
 */

#include "lidar_localization/publisher/tf_broadcaster.hpp"

namespace lidar_localization {
TFBroadCaster::TFBroadCaster(std::string frame_id, std::string child_frame_id) {
    transform_.frame_id_ = frame_id;
    transform_.child_frame_id_ = child_frame_id;
}

void TFBroadCaster::SendTransform(Eigen::Matrix4f pose, double time) {
    Eigen::Quaternionf q(pose.block<3,3>(0,0));
<<<<<<< HEAD
    ros::Time ros_time((float)time);
=======
    ros::Time ros_time(time);
>>>>>>> 124da3fd8c72597742a4c00e8aa60a9369b719e3
    transform_.stamp_ = ros_time;
    transform_.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
    transform_.setOrigin(tf::Vector3(pose(0,3), pose(1,3), pose(2,3)));
    broadcaster_.sendTransform(transform_);
}
}