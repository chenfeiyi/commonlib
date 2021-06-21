/*
 * @Author: CHEN Feiyi
 * @LastEditTime: 2021-04-02 15:58:47
 * @Description: content
 */
#pragma once
#include <pcl/visualization/pcl_visualizer.h>

#include <opencv2/opencv.hpp>
#include <type_traits>
//**********************Image**************************

static int color[21][3] = {{255, 0, 0},     {255, 69, 0},    {255, 99, 71},
                           {255, 140, 0},   {255, 165, 0},   {238, 173, 14},
                           {255, 193, 37},  {255, 255, 0},   {255, 236, 139},
                           {202, 255, 112}, {0, 255, 0},     {84, 255, 159},
                           {127, 255, 212}, {0, 229, 238},   {152, 245, 255},
                           {178, 223, 238}, {126, 192, 238}, {28, 134, 238},
                           {0, 0, 255},     {72, 118, 255},  {122, 103, 238}};
static void UndistortImage(cv::Mat *in_img, Eigen::Matrix3d camera_matrix,
                           Eigen::Matrix<double, 5, 1> dist_coeffs) {
  // Undistort input image:
  cv::Mat intrinsic =
      (cv::Mat_<float>(3, 3) << camera_matrix(0, 0), camera_matrix(0, 1),
       camera_matrix(0, 2), camera_matrix(1, 0), camera_matrix(1, 1),
       camera_matrix(1, 2), camera_matrix(2, 0), camera_matrix(2, 1),
       camera_matrix(2, 2));
  cv::Mat temp = in_img->clone();
  cv::Mat distor(5, 1, CV_32FC1);
  distor = (cv::Mat_<float>(5, 1) << dist_coeffs(0), dist_coeffs(1),
            dist_coeffs(2), dist_coeffs(3), dist_coeffs(4));
  cv::undistort(temp, *in_img, intrinsic, distor);
}
static void UndistortImage(cv::Mat *in_img, cv::Mat camera_matrix,
                           cv::Mat dist_coeffs) {
  // Undistort input image:
  cv::Mat temp = in_img->clone();
  undistort(temp, *in_img, camera_matrix, dist_coeffs);
}
template <typename T>
static cv::Mat LidarPorj2Camera(const cv::Mat &img_in,
                                boost::shared_ptr<pcl::PointCloud<T> > inCloud,
                                Eigen::Matrix3d R_L2C, Eigen::Vector3d T_L2C,
                                Eigen::Matrix3d intrinsics) {
  pcl::PointCloud<pcl::PointXYZ> cloud2camera;
  Eigen::Matrix3d cameraCoor2WorldCoor;
  cv::Mat img_out = img_in.clone();
  cameraCoor2WorldCoor << 0, 0, 1, -1, 0, 0, 0, -1, 0;
  float minimum_dis = 5;
  float dis_resolu = 0.2;
  for (int i = 0; i < inCloud->size(); i++) {
    pcl::PointXYZ pt;
    Eigen::Vector3d cloudpoint;
    Eigen::Vector3d camerapoint;
    Eigen::Vector3d pixelpoint;
    cloudpoint << inCloud->points[i].x, inCloud->points[i].y,
        inCloud->points[i].z;
    camerapoint = R_L2C * cloudpoint + T_L2C;
    // camerapoint = cameraCoor2WorldCoor.inverse() * cloudpoint;
    pixelpoint = intrinsics * camerapoint;
    int pixex = pixelpoint(0) / pixelpoint(2);
    int pixey = pixelpoint(1) / pixelpoint(2);

    if (pixex > 2 && pixex < img_in.size[1] && pixey > 2 &&
        pixey < img_in.size[0] - 2 && pixelpoint(2) > 0) {
      int color_order = static_cast<int>((cloudpoint(0) - minimum_dis) / dis_resolu);
      if (color_order > 20)
        color_order = 20;
      else if (color_order < 0)
        color_order = 0;
      cv::Vec3b ccc = cv::Vec3b(color[color_order][2], color[color_order][1],
                                color[color_order][0]);

      img_out.at<cv::Vec3b>(pixey, pixex) = ccc;
      img_out.at<cv::Vec3b>(pixey + 1, pixex + 1) = ccc;
      img_out.at<cv::Vec3b>(pixey - 1, pixex - 1) = ccc;
      img_out.at<cv::Vec3b>(pixey + 1, pixex) = ccc;
      img_out.at<cv::Vec3b>(pixey, pixex + 1) = ccc;
      img_out.at<cv::Vec3b>(pixey - 1, pixex) = ccc;
      img_out.at<cv::Vec3b>(pixey, pixex - 1) = ccc;
    }
  }
  return img_out;
}

//**********************PCL**************************
template <typename pointT>
static void VisualizeCloud(std::string windowname,
                           boost::shared_ptr<pcl::PointCloud<pointT> > cloud) {
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
      new pcl::visualization::PCLVisualizer(windowname));
  if (std::is_same<pcl::PointXYZI, pointT>::value) {
    pcl::visualization::PointCloudColorHandlerGenericField<pointT> fildColor(
        cloud, "intensity");
    viewer->addPointCloud<pointT>(cloud, fildColor, "crop");
  } else {
    viewer->addPointCloud<pointT>(cloud);
  }
  viewer->addCoordinateSystem(1);
  viewer->setBackgroundColor(0, 0, 0);
  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(1000));
  }
}

static void VisualizeOdom(std::string windowname,
                          std::vector<Eigen::Vector3d> odomt1,
                          std::vector<Eigen::Vector3d> odomt2,
                          Eigen::Matrix4d T=Eigen::Matrix4d::Identity(),
                          double scale=1) {

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
      new pcl::visualization::PCLVisualizer(windowname));

  viewer->addCoordinateSystem(0.2);
  viewer->setBackgroundColor(0, 0, 0);
  for (size_t i = 0; i < odomt1.size()-1; i++)
  {
    pcl::PointXYZ pt1,pt2;

    Eigen::Vector3d pt1vec = scale * odomt1[i];
    Eigen::Vector3d pt2vec = scale *  odomt1[i+1];
    pt1.x=pt1vec[0];
    pt1.y=pt1vec[1];
    pt1.z=pt1vec[2];
    pt2.x=pt2vec[0];
    pt2.y=pt2vec[1];
    pt2.z=pt2vec[2];
    viewer->addLine(pt1,pt2,1,0,0,std::to_string(i));
    viewer->addSphere(pt1,0.05,1,1,1,std::to_string(i+2*odomt1.size()));

    pt1vec= T.block<3, 3>(0, 0) * odomt2[i] + T.block<3, 1>(0, 3);
    pt2vec = T.block<3, 3>(0, 0) * odomt2[i+1] + T.block<3, 1>(0, 3);
    pt1.x=pt1vec[0];
    pt1.y=pt1vec[1];
    pt1.z=pt1vec[2];
    pt2.x=pt2vec[0];
    pt2.y=pt2vec[1];
    pt2.z=pt2vec[2];
    viewer->addLine(pt1,pt2,0,1,0,std::to_string(i+odomt1.size()));
    viewer->addSphere(pt1,0.05,1,1,1,std::to_string(i+3*odomt1.size()));
  }
  viewer->addCoordinateSystem(0.2);
  viewer->setBackgroundColor(0, 0, 0);
  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(1000));
  }
}
