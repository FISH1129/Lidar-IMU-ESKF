#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/filtering/kitti_filtering_flow.hpp"
#include "lidar_localization/tools/file_manager.hpp"

#include "glog/logging.h"
#include <ostream>

namespace lidar_localization{

KITTIFilteringFlow::KITTIFilteringFlow(ros::NodeHandle &nh, std::string cloud_topic){
    // subscriber:
    // a. IMU rawmeasurement:
    imu_raw_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu/extract", 1000000);
    // b. undistorted Velodyne measurement:
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, cloud_topic, 100000);
    // c. IMU synced measurement:
    imu_synced_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/synced_imu", 100000);
    // d. synced GNSS-odo measurement:
    pos_vel_sub_ptr_ = std::make_shared<PosVelSubscriber>(nh, "synced_pos_vel", 100000);
    // e. lidar pose iin map frame:
    gnss_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/synced_gnss", 100000);
    // f. lidar to imu tf:
    lidar_to_imu_ptr_ = std::make_shared<TFListener>(nh, "/imu_link", "/velo_link");

    // publisher:
    // *: 代表必须要发布的话题
    // a. global point cloud map:
    // global_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/global_map", "/map", 100);
    // b. local point cloud map:
    // local_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/local_map", "/map", 100);
    // c. current scan:
    // current_scan_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/current_scan", "/map", 100);
    // *d. estimated lidar pose in map frame:
    laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/laser_odom", "/map", "/lidar", 100);
    // *e. fused pose in map frame:
    fused_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/fused_odom", "/map", "/lidar", 100);
    // f. tf:
    laser_tf_pub_ptr_ = std::make_shared<TFBroadCaster>("/map", "/vehicle_link");

    filtering_ptr_ = std::make_shared<KITTIFiltering>();
}

bool KITTIFilteringFlow::Run(){
    if(!InitCalibration()){
        return false;
    }

    ReadData();

    while(HasData()){
        if (!HasInited()) {
            if(ValidLidarData()) {
                InitLocalization();
            }
        } else {
            // TODO: handle timestamp chaos in an more elegant way

            bool a = HasLidarData();
            std::cout << "HasLidarData():"<< a << std::endl;
            // bool b = ValidLidarData();
            // std::cout << "ValidLidarData():"<< b << std::endl;

            if (a && ValidLidarData()) {
                std::cout << "进入第二层" << std::endl;
                bool c = HasIMUData();
                std::cout << "HasIMUData():"<< c << std::endl;  

                if (c) {
                    std::cout << "进入第三层" << std::endl;
                    double time_diff = current_imu_raw_data_.time - current_imu_raw_data_.time;
                    
                    while (HasIMUData() && ValidIMUData() && current_imu_raw_data_.time < current_cloud_data_.time) {
                        std::cout << "××××××××××××××××××××××××进入UpdateLocalization之前×××××××××××××××××××××××" << std::endl;
                        std::cout << "××××××××current_imu_raw_data_time - current_cloud_data_.time××××××:"<< time_diff << std::endl;
                        UpdateLocalization();
                    }

                    if (current_imu_raw_data_.time >= current_cloud_data_.time) {
                        std::cout << "-----------------------imu_raw加入buff中供下一次使用---------------------------" << std::endl;
                        imu_raw_data_buff_.push_back(current_imu_raw_data_);
                    }
                }
                std::cout << "××××××××××××××××××××××××进入CorrectLocalization之前×××××××××××××××××××××××" << std::endl;
                CorrectLocalization();
            }

            if (HasIMUData() && ValidIMUData()) {
                std::cout << "××××××××××××××××××××××××没有lidar数据时的UpdateLocalization×××××××××××××××××××××××" << std::endl;
                UpdateLocalization();
            }
        }       
    }

    return true;
}

bool KITTIFilteringFlow::ReadData() {
  //
  // pipe raw IMU measurements into buffer:
  //
  imu_raw_sub_ptr_->ParseData(imu_raw_data_buff_);
  std::cout << "读数据之前个数：" << imu_raw_data_buff_.size() << std::endl;
  while (HasInited() && HasIMUData() &&
        imu_raw_data_buff_.front().time < filtering_ptr_->GetTime()) {
    imu_raw_data_buff_.pop_front(); // pop_front()直至时间戳对齐
  }

  // double diff_1 = imu_raw_data_buff_.front().time - filtering_ptr_->GetTime();
  // double diff_2 = filtering_ptr_->GetTime() - current_imu_raw_data_.time;
  std::cout << "读数据之后个数：" << imu_raw_data_buff_.size() << std::endl;
  std::cout << "imu_raw_buff.front()时间------>" << imu_raw_data_buff_.front().time << std::endl;
  std::cout << "filtering_ptr_->GetTime()---->" << filtering_ptr_->GetTime() << std::endl;
  std::cout << "current_imu_raw_data_.time--->" << current_imu_raw_data_.time << std::endl;

  //
  // pipe synced lidar-GNSS-IMU measurements into buffer:
  //
  cloud_sub_ptr_->ParseData(cloud_data_buff_);
  imu_synced_sub_ptr_->ParseData(imu_synced_data_buff_);
  pos_vel_sub_ptr_->ParseData(pos_vel_data_buff_);
  gnss_sub_ptr_->ParseData(gnss_data_buff_);

  return true;
}

bool KITTIFilteringFlow::HasInited(void) { return filtering_ptr_->HasInited(); }

bool KITTIFilteringFlow::HasData() {
  if (!HasInited()) {
    if (!HasLidarData()) {
      return false;
    }
  } else {
    if (!HasIMUData() && !HasLidarData()) {
      return false;
    }
  }
  return true;
}

bool KITTIFilteringFlow::ValidIMUData() {
  current_imu_raw_data_ = imu_raw_data_buff_.front();

  imu_raw_data_buff_.pop_front();

  return true;
}

bool KITTIFilteringFlow::ValidLidarData() {
  current_cloud_data_ = cloud_data_buff_.front();
  current_imu_synced_data_ = imu_synced_data_buff_.front();
  current_pos_vel_data_ = pos_vel_data_buff_.front();
  // current_gnss_data_ = gnss_data_buff_.front();

  double diff_imu_time = current_cloud_data_.time - current_imu_synced_data_.time;
  double diff_pos_vel_time = current_cloud_data_.time - current_pos_vel_data_.time;
  // double diff_gnss_time = current_cloud_data_.time - current_gnss_data_.time;

  if (diff_imu_time < -0.05 || diff_pos_vel_time < -0.05) {
    cloud_data_buff_.pop_front();
    return false;
  }

  if (diff_imu_time > 0.05) {
    imu_synced_data_buff_.pop_front();
    return false;
  }

  if (diff_pos_vel_time > 0.05) {
    pos_vel_data_buff_.pop_front();
    return false;
  }

  // if (diff_gnss_time > 0.05) {
  //   pos_vel_data_buff_.pop_front();
  //   return false;
  // }

  cloud_data_buff_.pop_front();
  imu_synced_data_buff_.pop_front();
  pos_vel_data_buff_.pop_front();
  // gnss_data_buff_.pop_front();

  return true;
}

bool KITTIFilteringFlow::InitCalibration() {
  // lookup imu pose in lidar frame:
  static bool calibration_received = false;

  if (!calibration_received) {
    if (lidar_to_imu_ptr_->LookupData(lidar_to_imu_)) {
      calibration_received = true;
    }
  }

  return calibration_received;
}

bool KITTIFilteringFlow::InitLocalization(void) {
  // ego vehicle velocity in body frame:
  Eigen::Vector3f init_vel = current_pos_vel_data_.vel;  ///??速度与位置不对应

  static int gnss_count = 0;
  if(!(gnss_count > 0)){
    current_gnss_data_ = gnss_data_buff_.at(gnss_count);//第0帧gnss,作为初始化
  }
  gnss_count++;
  // current_gnss_data_.pose = gnss_data_buff_.at(gnss_count);
  // gnss_data_buff_.front();

  // current_gnss_data_.pose = Eigen::Matrix4f::Identity();
  std::cout << "current_gnss_data_.pose" << std::endl << current_gnss_data_.pose  << std::endl;

  if(filtering_ptr_->Init(current_gnss_data_.pose, 
                          init_vel, 
                          current_imu_synced_data_)){
    //prompt:
    LOG(INFO) << "Gnss Localization Init Successed." << std::endl;
  }

  return true;
}

bool KITTIFilteringFlow::UpdateLocalization() {
  if (filtering_ptr_->Update(current_imu_raw_data_)) {
    PublishFusionOdom();
    std::cout << "aaaaaaaaaaaaaaaaa没有观测更新直接发布后验位姿" << std::endl;
    return true;
  }

  return false;
}

bool KITTIFilteringFlow::CorrectLocalization() {
  bool is_fusion_succeeded = filtering_ptr_->Correct(current_imu_synced_data_, 
                                                     current_cloud_data_,
                                                     current_pos_vel_data_, 
                                                     laser_pose_);
  
  PublishLidarOdom();
  std::cout << "bbbbbbbbbbbbbbbbb有观测更新发布观测位姿" << std::endl;
  std::cout << "是否融合成功is_fusion_succeeded:"<< is_fusion_succeeded << std::endl;

  if (is_fusion_succeeded) {
    PublishFusionOdom();
    std::cout << "ccccccccccccccc有观测更新发布融合后的位姿" << std::endl;
    // add to odometry output for evo evaluation:
    UpdateOdometry(current_cloud_data_.time);

    return true;
  }

  return false;
}

bool KITTIFilteringFlow::PublishLidarOdom() {
  // a. publish lidar odometry
  laser_odom_pub_ptr_->Publish(laser_pose_, current_cloud_data_.time);
  std::cout << "eeeeeeeeeeeeeeeeeeeeeeeeeeeeee发布ldiar的位姿" << std::endl;
  // b. publish current scan:
  // current_scan_pub_ptr_->Publish(filtering_ptr_->GetCurrentScan());

  return true;
}

bool KITTIFilteringFlow::PublishFusionOdom() {
  // get odometry from Kalman filter:
  filtering_ptr_->GetOdometry(fused_pose_, fused_vel_);
  // a. publish tf:
  laser_tf_pub_ptr_->SendTransform(fused_pose_, current_imu_raw_data_.time);
  // b. publish fusion odometry:
  fused_odom_pub_ptr_->Publish(fused_pose_, 
                               fused_vel_,
                               current_imu_raw_data_.time);
  std::cout << "ddddddddddddddddddddddddd发布融合后的位姿" << std::endl;

  return true;
}

bool KITTIFilteringFlow::UpdateOdometry(const double &time) {
  trajectory.time_.push_back(time);

  trajectory.fused_.push_back(fused_pose_);
  trajectory.lidar_.push_back(laser_pose_);

  ++trajectory.N;

  return true;
}

} //namespace lidar_localization