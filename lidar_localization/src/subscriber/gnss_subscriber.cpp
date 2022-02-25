/*
<<<<<<< HEAD
 * @Description:
=======
 * @Description: 
>>>>>>> 124da3fd8c72597742a4c00e8aa60a9369b719e3
 * @Author: Ren Qian
 * @Date: 2019-03-31 13:10:51
 */
#include "lidar_localization/subscriber/gnss_subscriber.hpp"

#include "glog/logging.h"

<<<<<<< HEAD
namespace lidar_localization
{
GNSSSubscriber::GNSSSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size) : nh_(nh)
{
  subscriber_ = nh_.subscribe(topic_name, buff_size, &GNSSSubscriber::msg_callback, this);
}

void GNSSSubscriber::msg_callback(const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr)
{
  buff_mutex_.lock();
  GNSSData gnss_data;
  gnss_data.time = nav_sat_fix_ptr->header.stamp.toSec();
  gnss_data.latitude = nav_sat_fix_ptr->latitude;
  gnss_data.longitude = nav_sat_fix_ptr->longitude;
  gnss_data.altitude = nav_sat_fix_ptr->altitude;
  gnss_data.status = nav_sat_fix_ptr->status.status;
  gnss_data.service = nav_sat_fix_ptr->status.service;

  new_gnss_data_.push_back(gnss_data);
  buff_mutex_.unlock();
}

//数据读入gnss_data_buff容器
void GNSSSubscriber::ParseData(std::deque<GNSSData>& gnss_data_buff)
{
  buff_mutex_.lock();
  if (new_gnss_data_.size() > 0)
  {
    gnss_data_buff.insert(gnss_data_buff.end(), new_gnss_data_.begin(), new_gnss_data_.end());
    new_gnss_data_.clear();
  }
  buff_mutex_.unlock();
=======
namespace lidar_localization {
GNSSSubscriber::GNSSSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size) 
    :nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &GNSSSubscriber::msg_callback, this);
}

void GNSSSubscriber::msg_callback(const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr) {
    buff_mutex_.lock();

    // convert ROS NavSatFix to GeographicLib compatible GNSS message:
    GNSSData gnss_data;
    gnss_data.time = nav_sat_fix_ptr->header.stamp.toSec();
    gnss_data.latitude = nav_sat_fix_ptr->latitude;
    gnss_data.longitude = nav_sat_fix_ptr->longitude;
    gnss_data.altitude = nav_sat_fix_ptr->altitude;
    gnss_data.status = nav_sat_fix_ptr->status.status;
    gnss_data.service = nav_sat_fix_ptr->status.service;
    // add new message to buffer:
    new_gnss_data_.push_back(gnss_data);

    buff_mutex_.unlock();
}

void GNSSSubscriber::ParseData(std::deque<GNSSData>& gnss_data_buff) {
    buff_mutex_.lock();

    // pipe all available measurements to output buffer:
    if (new_gnss_data_.size() > 0) {
        gnss_data_buff.insert(gnss_data_buff.end(), new_gnss_data_.begin(), new_gnss_data_.end());
        new_gnss_data_.clear();
    }

    buff_mutex_.unlock();
>>>>>>> 124da3fd8c72597742a4c00e8aa60a9369b719e3
}
}