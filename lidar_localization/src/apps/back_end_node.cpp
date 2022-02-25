/*
 * @Description: 前端里程计的node文件
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:56:27
 */
#include <ros/ros.h>
#include "glog/logging.h"

#include <lidar_localization/optimizeMap.h>
#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/mapping/back_end/back_end_flow.hpp"

using namespace lidar_localization;

std::shared_ptr<BackEndFlow> _back_end_flow_ptr;
bool _need_optimize_map = false;

bool optimize_map_callback(optimizeMap::Request& request, optimizeMap::Response& response)
{
  _need_optimize_map = true;
  response.succeed = true;
  return response.succeed;
}

int main(int argc, char* argv[])
{
  google::InitGoogleLogging(argv[0]);
  FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
  FLAGS_alsologtostderr = 1;

  ros::init(argc, argv, "back_end_node");
  ros::NodeHandle nh;

  std::string cloud_topic, laser_odom_topic, fused_odom_topic;
  //nh.param<std::string>("cloud_topic", cloud_topic, "/points_noground_synced"); //获取参数服务器的参数
  nh.param<std::string>("cloud_topic", cloud_topic, "/synced_cloud");
  nh.param<std::string>("laser_odom_topic", laser_odom_topic, "/laser_odom");
  nh.param<std::string>("fused_odom_topic", fused_odom_topic, "/fused_odom");

  ros::ServiceServer service = nh.advertiseService("optimize_map", optimize_map_callback);
  _back_end_flow_ptr = std::make_shared<BackEndFlow>(nh, cloud_topic, laser_odom_topic, fused_odom_topic);

  ros::Rate rate(100);
  while (ros::ok())
  {
    ros::spinOnce();

    _back_end_flow_ptr->Run(); //正常优化

    if (_need_optimize_map)
    {
      _back_end_flow_ptr->ForceOptimize(); //强制优化一次
      _need_optimize_map = false;
    }

    rate.sleep();
  }

  return 0;
}