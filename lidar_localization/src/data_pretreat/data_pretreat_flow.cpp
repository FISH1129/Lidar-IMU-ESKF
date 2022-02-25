/*
 * @Description: 数据预处理模块，包括时间同步、点云去畸变等
 * @Author: Ren Qian
 * @Date: 2020-02-10 08:38:42
 */
#include <ros/ros.h>
#include <ros/package.h>
#include "glog/logging.h"

#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/data_pretreat/data_pretreat_flow.hpp"

namespace lidar_localization
{
sretreatFlow::DataPretreatFlow(ros::NodeHandle& nh, std::string cloud_topic)
  {
    // subscriber
    // a. velodyne measurement:
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 100000);
    // b. OXTS IMU:
    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu", 1000000);
    // c. OXTS velocity:
    velocity_sub_ptr_ = std::make_shared<VelocitySubscriber>(nh, "/kitti/oxts/gps/vel", 1000000);
    // d. OXTS GNSS:
    gnss_sub_ptr_ = std::make_shared<GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 1000000);
    lidar_to_imu_ptr_ = std::make_shared<TFListener>(nh, "/imu_link", "/velo_link");

    // publisher
    cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, cloud_topic, "/velo_link", 100);
    imu_pub_ptr_ = std::make_shared<IMUPublisher>(nh, "/synced_imu", "/imu_link", 100);
    pos_vel_pub_ptr_ = std::make_shared<PosVelPublisher>(nh, "/synced_pos_vel", "/map", "/imu_link", 100);
    gnss_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/synced_gnss", "/map", "/velo_link", 100);

    distortion_adjust_ptr_ = std::make_shared<DistortionAdjust>();

    //基于地图的定位时：定位加载文件初始化 纬经高
    ros::NodeHandle nh_param("~");
    nh_param.param<bool>("only_localization", only_localization_, false);
    if (only_localization_)
    {
      if (!InitGNSSFromFile())
      {
        std::cout << "Error: 地图原点经纬度坐标文件不存在，请先建图生成！" << std::endl;
        return;
      }
    }
  }

bool DataPretreatFlow::Run()
{
    if (!ReadData())
        return false;

    if (!InitCalibration())
        return false;

    if (!only_localization_)
    {
        if (!InitGNSS())
          return false;
    }

    while (HasData())
    {
        if (!ValidData())
          continue;

        TransformData();
        PublishData();
    }

    return true;
}

bool DataPretreatFlow::ReadData()
{
  cloud_sub_ptr_->ParseData(cloud_data_buff_);

  static std::deque<IMUData> unsynced_imu_;
  static std::deque<VelocityData> unsynced_velocity_;
  static std::deque<GNSSData> unsynced_gnss_;

  imu_sub_ptr_->ParseData(unsynced_imu_);
  velocity_sub_ptr_->ParseData(unsynced_velocity_);
  gnss_sub_ptr_->ParseData(unsynced_gnss_);

  if (cloud_data_buff_.size() == 0)
    return false;

  double cloud_time = cloud_data_buff_.front().time;
  bool valid_imu = IMUData::SyncData(unsynced_imu_, imu_data_buff_, cloud_time);
  bool valid_velocity = VelocityData::SyncData(unsynced_velocity_, velocity_data_buff_, cloud_time);
  bool valid_gnss = GNSSData::SyncData(unsynced_gnss_, gnss_data_buff_, cloud_time);

  static bool sensor_inited = false;
  if (!sensor_inited)
  {
    if (!valid_imu || !valid_velocity || !valid_gnss)
    {
      cloud_data_buff_.pop_front();
      return false;
    }
    sensor_inited = true;
  }

  return true;
}

bool DataPretreatFlow::InitCalibration()
{
  static bool calibration_received = false;
  if (!calibration_received)
  {
    if (lidar_to_imu_ptr_->LookupData(lidar_to_imu_))
    {
      calibration_received = true;
    }
  }

  return calibration_received;
}

bool DataPretreatFlow::InitGNSS()
{
  static bool gnss_inited = false;
  if (!gnss_inited)
  {
    GNSSData gnss_data = gnss_data_buff_.front();
    gnss_data.InitOriginPosition();
    gnss_inited = true;

    //建图保存初始处的 纬经高
    std::string map_origion_gnss_file = ros::package::getPath("lidar_localization") + "/localization_data/"
                                                                                      "map_origion.txt";
    std::string localization_folder = ros::package::getPath("lidar_localization") + "/localization_data";
    if (access(localization_folder.c_str(), F_OK) != 0)
    {
      std::string cmd_str = "mkdir -p " + localization_folder;
      int ret = system(cmd_str.c_str());
      if (ret != 0)
      {
      }
    }
    std::ofstream gnss_file;
    gnss_file.open(map_origion_gnss_file);

    gnss_file << std::setprecision(15) << gnss_data.time << " " 
                                       << gnss_data.latitude << " " 
                                       << gnss_data.longitude << " " 
                                       << gnss_data.altitude << " " 
                                       << gnss_data.status << " " 
                                       << gnss_data.service << std::endl;
    gnss_file.close();
  }

  return gnss_inited;
}

bool DataPretreatFlow::InitGNSSFromFile()
{
  std::string map_origion_gnss_file = ros::package::getPath("lidar_localization") + "/localization_data/"
                                                                                    "map_origion.txt";
  std::ifstream origion_read_file;
  origion_read_file.open(map_origion_gnss_file);
  if (origion_read_file.is_open())
  {
    GNSSData init_map_gnss_data;
    std::string lidae_data;
    while (getline(origion_read_file, lidae_data))
    {
      std::istringstream ss(lidae_data);
      double word_data;
      int i = 0;
      while (ss >> word_data)
      {
        if (i == 0)
        {
          init_map_gnss_data.time = word_data;
        }
        else if (i == 1)
        {
          init_map_gnss_data.latitude = word_data;
        }
        else if (i == 2)
        {
          init_map_gnss_data.longitude = word_data;
        }
        else if (i == 3)
        {
          init_map_gnss_data.altitude = word_data;
        }
        else if (i == 4)
        {
          init_map_gnss_data.status = (int)word_data;
        }
        else if (i == 5)
        {
          init_map_gnss_data.service = (int)word_data;
        }
        i++;
      }
      init_map_gnss_data.InitOriginPosition();
      std::cout << "init map_gnss is: " << std::setprecision(15) << init_map_gnss_data.time << " "
                << init_map_gnss_data.latitude << " " << init_map_gnss_data.longitude << " "
                << init_map_gnss_data.altitude << " " << init_map_gnss_data.status << " " << init_map_gnss_data.service
                << std::endl;
    }
    return true;
  }
  return false;
}

bool DataPretreatFlow::HasData()
{
  if (cloud_data_buff_.size() == 0)
    return false;
  if (imu_data_buff_.size() == 0)
    return false;
  if (velocity_data_buff_.size() == 0)
    return false;
  if (gnss_data_buff_.size() == 0)
    return false;

  return true;
}

bool DataPretreatFlow::ValidData()
{
  current_cloud_data_ = cloud_data_buff_.front();
  current_imu_data_ = imu_data_buff_.front();
  current_velocity_data_ = velocity_data_buff_.front();
  current_gnss_data_ = gnss_data_buff_.front();

  double diff_imu_time = current_cloud_data_.time - current_imu_data_.time;
  double diff_velocity_time = current_cloud_data_.time - current_velocity_data_.time;
  double diff_gnss_time = current_cloud_data_.time - current_gnss_data_.time;

  // std::cout << "data_pretreat当前帧点云时间："     << current_cloud_data_.time << std::endl;
  // std::cout << "data_pretreat当前帧同步的IMU时间:" << current_imu_data_.time << std::endl;
  // std::cout << "data_pretreat当前帧pos_vel时间:"  << current_velocity_data_.time << std::endl;
  // std::cout << "data_pretreat当前帧gnss时间:"     << current_gnss_data_.time << std::endl;

  // std::cout << "data_pretreat同步的IMU时间diff:" << diff_imu_time << std::endl;
  // std::cout << "data_pretreatpos_vel时间diff:"  << diff_velocity_time << std::endl;
  // std::cout << "data_pretreatgnss时间diff:"     << diff_gnss_time << std::endl;

  if (diff_imu_time < -0.05 || diff_velocity_time < -0.05 || diff_gnss_time < -0.05)
  {
    cloud_data_buff_.pop_front();
    return false;
  }

  if (diff_imu_time > 0.05)
  {
    imu_data_buff_.pop_front();
    return false;
  }

  if (diff_velocity_time > 0.05)
  {
    velocity_data_buff_.pop_front();
    return false;
  }

  if (diff_gnss_time > 0.05)
  {
    gnss_data_buff_.pop_front();
    return false;
  }

  cloud_data_buff_.pop_front();
  imu_data_buff_.pop_front();
  velocity_data_buff_.pop_front();
  gnss_data_buff_.pop_front();

  return true;
}

bool DataPretreatFlow::TransformData()
{
  // a. get reference pose:
  gnss_pose_ = Eigen::Matrix4f::Identity();
  // get position from GNSS
  current_gnss_data_.UpdateXYZ();
  gnss_pose_(0, 3) = current_gnss_data_.local_E;
  gnss_pose_(1, 3) = current_gnss_data_.local_N;
  gnss_pose_(2, 3) = current_gnss_data_.local_U;
  // get orientation from IMU:
  gnss_pose_.block<3, 3>(0, 0) = current_imu_data_.GetOrientationMatrix();
  // this is lidar pose in GNSS/map frame:
  gnss_pose_ *= lidar_to_imu_;
  
  // b. set synced pos vel
  pos_vel_.pos.x() = current_gnss_data_.local_E;
  pos_vel_.pos.y() = current_gnss_data_.local_N;
  pos_vel_.pos.z() = current_gnss_data_.local_U;

  pos_vel_.vel.x() = current_velocity_data_.linear_velocity.x;
  pos_vel_.vel.y() = current_velocity_data_.linear_velocity.y;
  pos_vel_.vel.z() = current_velocity_data_.linear_velocity.z;

  // c. motion compensation for lidar measurements:
  //数据中的速度是IMU的速度，想要的是lidar的速度
  current_velocity_data_.TransformCoordinate(lidar_to_imu_);
  //使用去除畸变
  // distortion_adjust_ptr_->SetMotionInfo(0.1, current_velocity_data_);
  // distortion_adjust_ptr_->AdjustCloud(current_cloud_data_.cloud_ptr, current_cloud_data_.cloud_ptr);

  return true;
}

bool DataPretreatFlow::PublishData()
{
  cloud_pub_ptr_->Publish(current_cloud_data_.cloud_ptr, current_cloud_data_.time);     //lidar系下的点云
  imu_pub_ptr_->Publish(current_imu_data_, current_cloud_data_.time);                   //IMU系下的姿态、角速度、线加速度
  pos_vel_pub_ptr_->Publish(pos_vel_, current_cloud_data_.time);                        //IMU系相对于世界系map的角速度、线速度和位置在body下的表示

  // this synced odometry has the following info:
  // a. lidar frame's pose in map
  // b. lidar frame's velocity
  gnss_pub_ptr_->Publish(gnss_pose_, current_velocity_data_, current_cloud_data_.time); //lidar系相对与map的位姿、角速度、线速度

  return true;
}
}  // namespace lidar_localization
