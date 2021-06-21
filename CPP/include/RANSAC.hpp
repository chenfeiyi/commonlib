/*
 * @Author: CHEN Feiyi
 * @LastEditTime: 2021-04-25 12:57:42
 * @Description: content
 */
#pragma once
#include <pcl/io/io.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <memory>
#include <opencv2/opencv.hpp>
template <typename PointT>
static bool Plane3DRANSAC(boost::shared_ptr<pcl::PointCloud<PointT> > cloud_in,
                          Eigen::Vector4d *equation,double threshold=0.02,bool isOverride=false) {
  // inliers表示误差能容忍的点 记录的是点云的序号
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  // 创建一个分割器
  pcl::SACSegmentation<PointT> seg;
  // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
  seg.setOptimizeCoefficients(true);
  // Mandatory-设置目标几何形状
  seg.setModelType(pcl::SACMODEL_PLANE);
  // 分割方法：随机采样法
  seg.setMethodType(pcl::SAC_RANSAC);
  // 设置误差容忍范围，也就是我说过的阈值
  seg.setDistanceThreshold(threshold);
  // 输入点云
  try {
    seg.setInputCloud(cloud_in);
    // 分割点云
    seg.segment(*inliers, *coefficients);
    *equation << coefficients->values[0], coefficients->values[1],
        coefficients->values[2], coefficients->values[3];
    if ((*equation)(3) < 0) *equation = -*equation;
    *equation = *equation / equation->block<3, 1>(0, 0).norm();
    if(isOverride)
      pcl::copyPointCloud(*cloud_in, *inliers, *cloud_in);
    return true;
  } catch (...) {
    std::cout << "Point3DRansac: error to calculate coefficient!" << std::endl;
    return false;
  }
}
