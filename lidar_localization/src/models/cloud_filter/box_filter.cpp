/*
 * @Description: 从点云中截取一个立方体部分
 * @Author: Ren Qian
 * @Date: 2019-03-12 23:38:31
 */
#include <vector>
#include <iostream>
#include "glog/logging.h"

#include "lidar_localization/models/cloud_filter/box_filter.hpp"

<<<<<<< HEAD
namespace lidar_localization
{
BoxFilter::BoxFilter(YAML::Node node)
{
=======
namespace lidar_localization {
BoxFilter::BoxFilter(YAML::Node node) {
>>>>>>> 124da3fd8c72597742a4c00e8aa60a9369b719e3
    size_.resize(6);
    edge_.resize(6);
    origin_.resize(3);

<<<<<<< HEAD
    for (size_t i = 0; i < size_.size(); i++)
    {
=======
    for (size_t i = 0; i < size_.size(); i++) {
>>>>>>> 124da3fd8c72597742a4c00e8aa60a9369b719e3
        size_.at(i) = node["box_filter_size"][i].as<float>();
    }
    SetSize(size_);
}

<<<<<<< HEAD
bool BoxFilter::Filter(const CloudData::CLOUD_PTR &input_cloud_ptr,
                       CloudData::CLOUD_PTR &output_cloud_ptr)
{
=======
bool BoxFilter::Filter(const CloudData::CLOUD_PTR& input_cloud_ptr,
                       CloudData::CLOUD_PTR& output_cloud_ptr) {
>>>>>>> 124da3fd8c72597742a4c00e8aa60a9369b719e3
    output_cloud_ptr->clear();
    pcl_box_filter_.setMin(Eigen::Vector4f(edge_.at(0), edge_.at(2), edge_.at(4), 1.0e-6));
    pcl_box_filter_.setMax(Eigen::Vector4f(edge_.at(1), edge_.at(3), edge_.at(5), 1.0e6));
    pcl_box_filter_.setInputCloud(input_cloud_ptr);
    pcl_box_filter_.filter(*output_cloud_ptr);

    return true;
}

<<<<<<< HEAD
void BoxFilter::SetSize(std::vector<float> size)
{
    size_ = size;
    std::cout << "Box Filter 的尺寸为：" << std::endl
=======
void BoxFilter::SetSize(std::vector<float> size) {
    size_ = size;
    std::cout << "Box Filter params:" << std::endl
>>>>>>> 124da3fd8c72597742a4c00e8aa60a9369b719e3
              << "min_x: " << size.at(0) << ", "
              << "max_x: " << size.at(1) << ", "
              << "min_y: " << size.at(2) << ", "
              << "max_y: " << size.at(3) << ", "
              << "min_z: " << size.at(4) << ", "
              << "max_z: " << size.at(5)
<<<<<<< HEAD
              << std::endl
              << std::endl;

    CalculateEdge();
}

void BoxFilter::SetOrigin(std::vector<float> origin)
{
=======
              << std::endl << std::endl;
    
    CalculateEdge();
}

void BoxFilter::SetOrigin(std::vector<float> origin) {
>>>>>>> 124da3fd8c72597742a4c00e8aa60a9369b719e3
    origin_ = origin;
    CalculateEdge();
}

<<<<<<< HEAD
void BoxFilter::CalculateEdge()
{
    for (size_t i = 0; i < origin_.size(); ++i)
    {
=======
void BoxFilter::CalculateEdge() {
    for (size_t i = 0; i < origin_.size(); ++i) {
>>>>>>> 124da3fd8c72597742a4c00e8aa60a9369b719e3
        edge_.at(2 * i) = size_.at(2 * i) + origin_.at(i);
        edge_.at(2 * i + 1) = size_.at(2 * i + 1) + origin_.at(i);
    }
}

<<<<<<< HEAD
std::vector<float> BoxFilter::GetEdge()
{
    return edge_;
}
} // namespace lidar_localization
=======
std::vector<float> BoxFilter::GetEdge() {
    return edge_;
}
} // namespace lidar_slam
>>>>>>> 124da3fd8c72597742a4c00e8aa60a9369b719e3
