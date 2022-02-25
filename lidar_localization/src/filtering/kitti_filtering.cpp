#include "lidar_localization/filtering/kitti_filtering.hpp"

#include "glog/logging.h"
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/models/cloud_filter/no_filter.hpp"
#include "lidar_localization/models/cloud_filter/voxel_filter.hpp"
#include "lidar_localization/models/registration/ndt_registration.hpp"
#include "lidar_localization/models/registration/ndt_registration_manual.hpp"

#include "lidar_localization/models/kalman_filter/error_state_kalman_filter.hpp"

namespace lidar_localization{

KITTIFiltering::KITTIFiltering()
    : global_map_ptr_(new CloudData::CLOUD()),
      local_map_ptr_(new CloudData::CLOUD()),
      current_scan_ptr_(new CloudData::CLOUD()) {
    // load ROS config:
    InitWithConfig();
}

/*   gnss   初始化  pose ，  输入  matrix4d   current_gnss_pose_*/
bool KITTIFiltering::Init(const Eigen::Matrix4f &init_pose,
                          const Eigen::Vector3f &init_vel,
                          const IMUData &init_imu_data) {
    if (SetInitGNSS(init_pose)) {
        current_vel_ = init_vel;

        kalman_filter_ptr_->Init(current_vel_.cast<double>(), init_imu_data);

        return true;
    }

    return false;
}

bool KITTIFiltering::Update(const IMUData &imu_data) {
  if (kalman_filter_ptr_->Update(imu_data)) {
    kalman_filter_ptr_->GetOdometry(current_pose_, current_vel_);
    return true;
  }

  return false;
}

bool KITTIFiltering::Correct(const IMUData &imu_data,
                             const CloudData &cloud_data,
                             const PosVelData &pos_vel_data,
                             Eigen::Matrix4f &cloud_pose) {
  current_frame_.cloud_data.time = cloud_data.time;
  // remove invalid measurements:
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr, 
                               *current_frame_.cloud_data.cloud_ptr,
                               indices);

  // downsample:
  CloudData::CLOUD_PTR filtered_cloud_ptr(new CloudData::CLOUD());
  frame_filter_ptr_->Filter(current_frame_.cloud_data.cloud_ptr, filtered_cloud_ptr);

  static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
  static Eigen::Matrix4f last_pose = init_pose_;
  static Eigen::Matrix4f predict_pose = init_pose_;
  static Eigen::Matrix4f last_key_frame_pose = init_pose_;

  if (!has_inited_) {
    predict_pose = current_gnss_pose_;
  }

  // 局部地图没有关键帧，代表第一帧
  // 此时把当前帧的数据作为第一个关键帧，并更新局部地图容器和全局地图容器
  if (local_map_frames_.size() == 0) {
    current_frame_.pose = init_pose_;
    UpdateWithNewFrame(current_frame_);
    cloud_pose = current_frame_.pose;

    std::cout << "雷达计算的观测位姿  初始："  << std::endl;
    std::cout << current_frame_.pose  << std::endl;

    // set lidar measurement:
    current_measurement_.time = cloud_data.time;
    current_measurement_.T_nb = (init_pose_.inverse() * cloud_pose).cast<double>();
    current_measurement_.v_b = pos_vel_data.vel.cast<double>();
    current_measurement_.w_b = Eigen::Vector3d(imu_data.angular_velocity.x, 
                                               imu_data.angular_velocity.y,
                                               imu_data.angular_velocity.z);

      // Kalman correction:
    if (kalman_filter_ptr_->Correct(imu_data, 
                                    KalmanFilter::MeasurementType::POSE,
                                    current_measurement_)) {
        kalman_filter_ptr_->GetOdometry(current_pose_, current_vel_);
        
        std::cout << "融合后的位姿："  << std::endl;
        std::cout << current_pose_  << std::endl; 
        std::cout << "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"  << std::endl;
        
        return true;
    }  

    return false;
  }

  // 不是第一帧，正常matching:
  CloudData::CLOUD_PTR result_cloud_ptr(new CloudData::CLOUD());
  registration_ptr_->ScanMatch(filtered_cloud_ptr, predict_pose,
                               result_cloud_ptr, current_frame_.pose);
  cloud_pose = current_frame_.pose;
  
  std::cout << "雷达计算的观测位姿："  << std::endl;
  std::cout << current_frame_.pose  << std::endl;
  
  pcl::transformPointCloud(*cloud_data.cloud_ptr, 
                           *current_scan_ptr_,
                           cloud_pose);

  // update predicted pose:
  step_pose = last_pose.inverse() * current_frame_.pose;
  predict_pose = current_frame_.pose * step_pose;
  last_pose = current_frame_.pose;

  if (fabs(last_key_frame_pose(0,3) - current_frame_.pose(0,3)) + 
      fabs(last_key_frame_pose(1,3) - current_frame_.pose(1,3)) +
      fabs(last_key_frame_pose(2,3) - current_frame_.pose(2,3)) > key_frame_distance_) {
      UpdateWithNewFrame(current_frame_);
      last_key_frame_pose = current_frame_.pose;
  }
  // set lidar measurement:
  current_measurement_.time = cloud_data.time;
  current_measurement_.T_nb = (init_pose_.inverse() * cloud_pose).cast<double>();  // ？？？？？？？？？？为什么是求逆运算
  current_measurement_.v_b = pos_vel_data.vel.cast<double>();
  current_measurement_.w_b = Eigen::Vector3d(imu_data.angular_velocity.x, 
                                             imu_data.angular_velocity.y,
                                             imu_data.angular_velocity.z);
  
  // Kalman correction:
  if (kalman_filter_ptr_->Correct(imu_data, 
                                  KalmanFilter::MeasurementType::POSE,
                                  current_measurement_)) {
    kalman_filter_ptr_->GetOdometry(current_pose_, current_vel_);

    std::cout << "融合后的位姿："  << std::endl;
    std::cout << current_pose_  << std::endl; 
    std::cout << "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"  << std::endl;

    return true;
  }

  return false;
}

bool KITTIFiltering::UpdateWithNewFrame(const Frame& new_key_frame) {
    Frame key_frame = new_key_frame;
    // 这一步的目的是为了把关键帧的点云保存下来
    // 由于用的是共享指针，所以直接复制只是复制了一个指针而已
    // 此时无论你放多少个关键帧在容器里，这些关键帧点云指针都是指向的同一个点云
    key_frame.cloud_data.cloud_ptr.reset(new CloudData::CLOUD(*new_key_frame.cloud_data.cloud_ptr));
    CloudData::CLOUD_PTR transformed_cloud_ptr(new CloudData::CLOUD());
    
    // 更新局部地图
    local_map_frames_.push_back(key_frame);
    while (local_map_frames_.size() > static_cast<size_t>(local_frame_num_)) {
        local_map_frames_.pop_front();
    }
    local_map_ptr_.reset(new CloudData::CLOUD());
    for (size_t i = 0; i < local_map_frames_.size(); ++i) {
        pcl::transformPointCloud(*local_map_frames_.at(i).cloud_data.cloud_ptr, 
                                 *transformed_cloud_ptr, 
                                 local_map_frames_.at(i).pose);

        *local_map_ptr_ += *transformed_cloud_ptr;
    }

    // 更新ndt匹配的目标点云
    // 关键帧数量还比较少的时候不滤波，因为点云本来就不多，太稀疏影响匹配效果
    if (local_map_frames_.size() < 10) {
        registration_ptr_->SetInputTarget(local_map_ptr_);
    } else {
        CloudData::CLOUD_PTR filtered_local_map_ptr(new CloudData::CLOUD());
        local_map_filter_ptr_->Filter(local_map_ptr_, filtered_local_map_ptr);
        registration_ptr_->SetInputTarget(filtered_local_map_ptr);
    }

    return true;
}

void KITTIFiltering::GetOdometry(Eigen::Matrix4f &pose, Eigen::Vector3f &vel) {
  pose = init_pose_ * current_pose_;
  vel  = init_pose_.block<3, 3>(0, 0) * current_vel_;
}

bool KITTIFiltering::InitWithConfig(void){
    std::string config_file_path = WORK_SPACE_PATH + "/config/filtering/kitti_filtering.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    LOG(INFO) << std::endl
              << "---------------Init IMU-Lidar Fusion for Localization and mapping---------"
              << std::endl;
    
    // a. init filters
    // InitFilters(config_node);
    // b. init map
    // InitGlobalMap(config_node);
    // c. init scan context manage
    //InitScanContextManager(config_node);
    // d. init front end
    //InitRegistration(registration_ptr_, config_node);
    // e. init fusion
    InitFusion(config_node);

    // init local map for front end matching:
    // ResetLocalMap(0.0, 0.0, 0.0);

    std::cout << "-----------------观测部分：Lidar里程计的初始化---------------" << std::endl;
    InitParam(config_node);
    InitRegistration(registration_ptr_, config_node);
    InitFilter("local_map", local_map_filter_ptr_, config_node);
    InitFilter("frame", frame_filter_ptr_, config_node);
    
    return true; 
}

bool KITTIFiltering::InitFusion(const YAML::Node &config_node) {
    // set up fusion method:
    CONFIG.FUSION_METHOD = config_node["fusion_method"].as<std::string>();
    if (CONFIG.FUSION_METHOD == "error_state_kalman_filter") {
        kalman_filter_ptr_ = std::make_shared<ErrorStateKalmanFilter>(config_node[CONFIG.FUSION_METHOD]);
        std::cout << "\tIMU与Lidar融合方式: " << "error_state_kalman_filter" << std::endl;
    } else {
        LOG(ERROR) << "Fusion method " << CONFIG.FUSION_METHOD << " NOT FOUND!";
        return false;
    }
    std::cout << "\tKITTI Localization Fusion Method: " << CONFIG.FUSION_METHOD
                << std::endl;

    return true;
}

bool KITTIFiltering::InitParam(const YAML::Node& config_node) {
    key_frame_distance_ = config_node["key_frame_distance"].as<float>();
    local_frame_num_    = config_node["local_frame_num"].as<int>();

    return true;
}

bool KITTIFiltering::InitRegistration(std::shared_ptr<RegistrationInterface> &registration_ptr,
                                      const YAML::Node &config_node) {
    std::string registration_method = config_node["registration_method"].as<std::string>();

    std::cout << "\tPoint Cloud Registration Method: " << registration_method << std::endl;

    if (registration_method == "NDT") {
        registration_ptr = std::make_shared<NDTRegistration>(config_node[registration_method]);
    }else if(registration_method == "NDTMANUAL"){
        registration_ptr = std::make_shared<NDTRegistrationManual>(config_node[registration_method]);
    }else {
        LOG(ERROR) << "未找到雷达位姿的配准方式： " << registration_method << " NOT FOUND!";
        return false;
    }

    return true;
}

bool KITTIFiltering::InitFilter(std::string filter_user,
                                std::shared_ptr<CloudFilterInterface> &filter_ptr,
                                const YAML::Node &config_node){
    std::string filter_method = config_node[filter_user + "_filter"].as<std::string>();

    std::cout << "\tFilter Method for " << filter_user << ": " << filter_method << std::endl;

    if (filter_method == "voxel_filter") {
        filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_method][filter_user]);
    } else if (filter_method == "no_filter") {
        filter_ptr = std::make_shared<NoFilter>();
    } else {
        LOG(ERROR) << "Filter method " << filter_method << " for " << filter_user << " NOT FOUND!";
        return false;
    }

    return true;
}

bool KITTIFiltering::SetInitGNSS(const Eigen::Matrix4f &gnss_pose) {
    static int gnss_cnt = 0;

    current_gnss_pose_ = gnss_pose;

    if(gnss_cnt == 0){          //gnss数据前几帧不准，取第4帧gnss，index = 3,完成初始化
        has_inited_ = true;
    }
    std::cout<<"设置初始GNSS："<< gnss_pose << std::endl;
    
    SetInitPose(gnss_pose);

    gnss_cnt++;

    return true;
}

bool KITTIFiltering::SetInitPose(const Eigen::Matrix4f &init_pose) {
    init_pose_ = init_pose;
    return true;
}

} // namespace lidar_localization