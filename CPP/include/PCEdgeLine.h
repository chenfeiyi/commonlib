#pragma once
#include <math.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include "RANSAC.hpp"
#include "Visualize.hpp"
#include "log.h"
struct MyPoint{
  Eigen::Vector3d XYZ;
  double alpha;
  bool operator<(const MyPoint pt2){
    return this->alpha<pt2.alpha;
  }
};
class PCEdgeLine
{
private:

public:
  typedef pcl::PointXYZI PointT;
  PCEdgeLine(/* args */);
  // template <class T>
  Eigen::Vector4d RemoveGround(boost::shared_ptr<pcl::PointCloud<PointT> > cloudIn,
                    boost::shared_ptr<pcl::PointCloud<PointT> > cloudOut,
                    Eigen::Vector3d groundN = Eigen::Vector3d(0,0,1));
  // only applicable for 16 lidar
  void EdgeExtraction(boost::shared_ptr<pcl::PointCloud<PointT> > cloudIn,
                      boost::shared_ptr<pcl::PointCloud<PointT> > cloudOut);
  void EdgeLineExtraction(boost::shared_ptr<pcl::PointCloud<PointT> > cloudIn,
                          std::vector<pcl::PointCloud<PointT> > *cloudOut);
  ~PCEdgeLine();
};
