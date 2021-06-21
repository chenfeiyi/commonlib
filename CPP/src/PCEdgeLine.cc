#include "PCEdgeLine.h"

PCEdgeLine::PCEdgeLine(/* args */)
{
}

PCEdgeLine::~PCEdgeLine()
{
}

// template <class T>
Eigen::Vector4d PCEdgeLine::RemoveGround(boost::shared_ptr<pcl::PointCloud<PointT> > cloudIn,
                              boost::shared_ptr<pcl::PointCloud<PointT> > cloudOut,
                              Eigen::Vector3d groundN) {
// ****ransac parameters****
  int sampleSize = 5;
  double ratio=0.2; // the ratio of potional ground points 
  double maxdistance = 0.1;
// transform z-axis of original point cloud to standard z-axis=[0,0,1]
  Eigen::Matrix4d rotM;
  Eigen::Vector4d planeEquation;
  Eigen::Vector3d zAxis(0,0,1);
  Eigen::Vector3d spinvector = groundN.cross(zAxis);
  spinvector.normalize();
  double maxIdx = ratio * cloudIn->size();
  double averageZ=0;
  double theta = acos(zAxis.dot(groundN));
  if (fabs(theta*180/M_PI)<5)
    rotM = Eigen::Matrix4d::Identity();
  else{
    rotM = Eigen::Matrix4d::Identity();
    Eigen::Quaterniond qR(cos(theta / 2), sin(theta / 2) * spinvector[0],
                          sin(theta / 2) * spinvector[1],
                          sin(theta / 2) * spinvector[2]);
    rotM.block<3,3>(0,0) = qR.toRotationMatrix();
  }
  pcl::transformPointCloud(*cloudIn,*cloudOut,rotM);
  std::sort(cloudOut->begin(), cloudOut->end(), [](pcl::PointXYZI pt1, pcl::PointXYZI pt2) { return pt1.z < pt2.z; });

  for (size_t i = 0; i < maxIdx; i++)
  {
    averageZ+=cloudOut->points[i].z;
  }
  averageZ = averageZ/maxIdx;
  pcl::PointCloud<PointT> suspectGround;
  for (size_t i = 0; i < cloudOut->size(); i++) {
    if(fabs(cloudOut->points[i].z-averageZ)<0.3){
      suspectGround.push_back(cloudOut->points[i]);
    } else if (cloudOut->points[i].z - averageZ > 1) {
      break;
    }
  }
  Plane3DRANSAC(suspectGround.makeShared(),&planeEquation,maxdistance);
  suspectGround.clear();
  for (size_t i = 0; i < cloudOut->size(); i++) {
    double distance = cloudOut->points[i].x * planeEquation[0] +
                      cloudOut->points[i].y * planeEquation[1] +
                      cloudOut->points[i].z * planeEquation[2] +
                      planeEquation[3];
    if(fabs(distance)<0.3){
      suspectGround.push_back(cloudOut->points[i]);
    }
  }
  Plane3DRANSAC(suspectGround.makeShared(),&planeEquation,maxdistance);
  suspectGround.clear();
  for (size_t i = 0; i < cloudOut->size(); i++) {
    double distance = cloudOut->points[i].x * planeEquation[0] +
                      cloudOut->points[i].y * planeEquation[1] +
                      cloudOut->points[i].z * planeEquation[2] +
                      planeEquation[3];
    if(fabs(distance)>0.3){
      suspectGround.push_back(cloudOut->points[i]);
    }
  }
  cloudOut->clear();
  Eigen::Matrix4d rotMTrans = rotM.transpose();
  pcl::transformPointCloud(suspectGround,*cloudOut,rotMTrans);
  std::cout<<"planeEquation: "<<planeEquation.transpose()<<std::endl;
  planeEquation.block<3,1>(0,0) = rotMTrans.block<3,3>(0,0)*planeEquation.block<3,1>(0,0);
  return planeEquation;  
}
// only applicable for 16 lidar
void PCEdgeLine::EdgeExtraction(boost::shared_ptr<pcl::PointCloud<PointT> > cloudIn,
                    boost::shared_ptr<pcl::PointCloud<PointT> > cloudOut){
  std::vector< std::vector<MyPoint> > lines_points;
  Eigen::Vector3d eigen_pt;
  int minLinePtsNum = 20;
  lines_points.resize(16);
  cloudOut->clear();
  //************* Cluster points according vertical angle and horizontal angle*****
  for (int i = 0; i < cloudIn->size(); i++) {
    double alpha = 0;
    eigen_pt << cloudIn->points[i].x, cloudIn->points[i].y,cloudIn->points[i].z;
    double theta = asin(eigen_pt(2) / eigen_pt.norm()) * 180 / M_PI;
    if (eigen_pt(0) > 0) {
      alpha = atan(eigen_pt(1) / eigen_pt(0)) * 180 / M_PI;
      if (alpha < 0) alpha = alpha + 360;
    } else if (eigen_pt(0) == 0 && eigen_pt(1) > 0) {
      alpha = 90;
    } else if (eigen_pt(0) == 0 && eigen_pt(1) < 0) {
      alpha = 270;
    } else if (eigen_pt(0) < 0) {
      alpha = atan(eigen_pt(1) / eigen_pt(0)) * 180 / M_PI + 180;
    }
    int ring = round((theta+15)/2);
    if (ring >= 0 && ring < 16) {
      MyPoint pt;
      pt.XYZ[0] = cloudIn->points[i].x;
      pt.XYZ[1] = cloudIn->points[i].y;
      pt.XYZ[2] = cloudIn->points[i].z;
      pt.alpha = alpha;
      lines_points[ring].push_back(pt);
    } else {
      std::cout << "find strange point and theta: " << theta << std::endl;
    }
  }
  for (int i = 0; i < lines_points.size(); i++) {
    if (lines_points[i].size() < 5) continue;
    std::sort(lines_points[i].begin(), lines_points[i].end());
    std::vector<int> idxChange;
    for (size_t j = 0; j < lines_points[i].size(); j++) {
      Eigen::Vector3d depth =
          lines_points[i][(j + 1) % (lines_points[i].size() - 1)].XYZ - lines_points[i][j].XYZ;
      if (depth.norm() > 0.2) {
        idxChange.push_back(j + 1);
      }
    }
    if(idxChange.size()<2) continue;
    for (size_t j = 0; j < idxChange.size(); j++) {
      Eigen::Vector3d endPt;
      Eigen::Vector3d lastEndPt;
      Eigen::Vector3d startPt;
      Eigen::Vector3d nextStartPt;
      Eigen::Vector2d startEdgeDir;
      Eigen::Vector2d endEdgeDir;
      if(j==0){
        if(idxChange.back()>lines_points[i].size()-1){
          startPt = lines_points[i][0].XYZ;
          lastEndPt = lines_points[i].back().XYZ;
          int  numPts= idxChange[j]-1;
          if(numPts<minLinePtsNum)
            continue;
        }else{
          lastEndPt = lines_points[i][idxChange.back() - 1].XYZ;
          startPt = lines_points[i][idxChange.back()].XYZ;
          int numPts = idxChange[j]-1+lines_points[i].size()-idxChange.back();
          if(numPts<minLinePtsNum)
            continue;
        }
        endPt =  lines_points[i][idxChange[j]-1].XYZ;
        nextStartPt = lines_points[i][idxChange[j]].XYZ;
      }else{
        startPt = lines_points[i][idxChange[j-1]].XYZ;
        endPt =  lines_points[i][idxChange[j]-1].XYZ;
        lastEndPt = lines_points[i][idxChange[j-1]-1].XYZ;
        if (idxChange[j]>lines_points[i].size()-1){
            nextStartPt = lines_points[i][0].XYZ;
        } else {
          nextStartPt = lines_points[i][idxChange[j]].XYZ;
        }
        int numPts = idxChange[j]-1 - idxChange[j-1];
        if(numPts<minLinePtsNum)
            continue;
      }
      startEdgeDir = lastEndPt.block<2,1>(0,0) - startPt.block<2,1>(0,0);
      endEdgeDir = nextStartPt.block<2,1>(0,0) - endPt.block<2,1>(0,0);
      startEdgeDir.normalized();
      endEdgeDir.normalized();

      if(startEdgeDir.dot(startPt.block<2,1>(0,0))/startPt.block<2,1>(0,0).norm()>-0.99){
        PointT pt;
        pt.intensity=0;
        pt.x = startPt[0], pt.y = startPt[1], pt.z = startPt[2];
        cloudOut->push_back(pt);
      }
      if(endEdgeDir.dot(endPt.block<2,1>(0,0))/endPt.block<2,1>(0,0).norm()>-0.99){
        PointT pt;
        pt.intensity=0;
        pt.x = endPt[0], pt.y = endPt[1], pt.z = endPt[2];
        cloudOut->push_back(pt);
      }
    }
  }
}


void PCEdgeLine::EdgeLineExtraction(boost::shared_ptr<pcl::PointCloud<PointT> > cloudIn,
                        std::vector<pcl::PointCloud<PointT> > *cloudOut){
  // divide point cloud into 6 section 
  double lineDis = 0.1;
  std::vector< std::vector<MyPoint> > sectors;
  std::vector<Eigen::Vector3d> cloudInEigen;
  std::vector<std::vector<int>> cloudLineGroup;
  std::map<int,std::vector<int>> idxGroupIdxMap;
  sectors.resize(6);
  for (int i = 0; i < cloudIn->size(); i++) {
    Eigen::Vector3d eigen_pt;
    double alpha = 0;
    eigen_pt << cloudIn->points[i].x, cloudIn->points[i].y,cloudIn->points[i].z;
    cloudInEigen.push_back(eigen_pt);
    if (eigen_pt(0) > 0) {
      alpha = atan(eigen_pt(1) / eigen_pt(0)) * 180 / M_PI;
      if (alpha < 0) alpha = alpha + 360;
    } else if (eigen_pt(0) == 0 && eigen_pt(1) > 0) {
      alpha = 90;
    } else if (eigen_pt(0) == 0 && eigen_pt(1) < 0) {
      alpha = 270;
    } else if (eigen_pt(0) < 0) {
      alpha = atan(eigen_pt(1) / eigen_pt(0)) * 180 / M_PI + 180;
    }
    if(alpha<60){
      MyPoint pt;
      pt.XYZ[0] = cloudIn->points[i].x;
      pt.XYZ[1] = cloudIn->points[i].y;
      pt.XYZ[2] = cloudIn->points[i].z;
      pt.alpha = alpha;
      sectors[0].push_back(pt);
    }else if(alpha>=60&&alpha<120){
      MyPoint pt;
      pt.XYZ[0] = cloudIn->points[i].x;
      pt.XYZ[1] = cloudIn->points[i].y;
      pt.XYZ[2] = cloudIn->points[i].z;
      pt.alpha = alpha;
      sectors[1].push_back(pt);
    }else if(alpha>=120&&alpha<180){
      MyPoint pt;
      pt.XYZ[0] = cloudIn->points[i].x;
      pt.XYZ[1] = cloudIn->points[i].y;
      pt.XYZ[2] = cloudIn->points[i].z;
      pt.alpha = alpha;
      sectors[2].push_back(pt);
    }else if(alpha>=180&&alpha<240){
      MyPoint pt;
      pt.XYZ[0] = cloudIn->points[i].x;
      pt.XYZ[1] = cloudIn->points[i].y;
      pt.XYZ[2] = cloudIn->points[i].z;
      pt.alpha = alpha;
      sectors[3].push_back(pt);
    }else if(alpha>=240&&alpha<300){
      MyPoint pt;
      pt.XYZ[0] = cloudIn->points[i].x;
      pt.XYZ[1] = cloudIn->points[i].y;
      pt.XYZ[2] = cloudIn->points[i].z;
      pt.alpha = alpha;
      sectors[4].push_back(pt);
    }else if(alpha>=300&&alpha<=360){
      MyPoint pt;
      pt.XYZ[0] = cloudIn->points[i].x;
      pt.XYZ[1] = cloudIn->points[i].y;
      pt.XYZ[2] = cloudIn->points[i].z;
      pt.alpha = alpha;
      sectors[5].push_back(pt);
    }
  }
//project it into 2D image, find line using hough transformation
  Eigen::Matrix3d lidar2Cam;
  lidar2Cam<<0,-1,0,0,0,-1,1,0,0;
  for (size_t i = 0; i < sectors.size(); i++) {
    if (sectors[i].size() < 2) continue;
    float theta = -(i*60.0f+30.0f)*M_PI/180.0f;
    Eigen::Quaterniond q(cos(theta/2),0.0,0.0,sin(theta/2));
    Eigen::Matrix3d rotM = q.toRotationMatrix();
    std::vector<Eigen::Vector3d> aftPts;
    double min_x,min_y;
    double max_x,max_y;
    for (size_t j = 0; j < sectors[i].size(); j++) {
      Eigen::Vector3d aftPt = lidar2Cam*rotM*sectors[i][j].XYZ;
      Eigen::Vector2d pixelPt = aftPt.block<2,1>(0,0)/aftPt[2];
      aftPts.push_back(aftPt);
      if(j==0){
        min_x = pixelPt[0];
        min_y = pixelPt[1];
        max_x = pixelPt[0];
        max_y = pixelPt[1];
      }else{
        if(min_x>pixelPt[0])
          min_x = pixelPt[0];
        if(min_y>pixelPt[1])
          min_y = pixelPt[1];
        if(max_x<pixelPt[0])
          max_x = pixelPt[0];
        if(max_y<pixelPt[1])
          max_y = pixelPt[1];
      }
    }
    Eigen::Matrix3d pseudoK;
    pseudoK<<100,0,-std::min(0.0,100*min_x)+21,0,100,-std::min(0.0,100*min_y)+21,0,0,1;
    cv::Mat pseudoImg((int)ceil(pseudoK(0,2)+100*max_x+20),(int)ceil(pseudoK(1,2)+100*max_y+20),CV_8UC1,cv::Scalar(0));
    cv::Mat pseudoImgIdx((int)ceil(pseudoK(0,2)+100*max_x+20),(int)ceil(pseudoK(1,2)+100*max_y+20),CV_16UC1,cv::Scalar(0));
    for (int j = 0; j < aftPts.size(); j++) {
      Eigen::Vector3d pt = pseudoK*aftPts[j];
      Eigen::Vector2d pixelPt = pt.block<2,1>(0,0)/aftPts[j][2];
      pseudoImg.at<uchar>((int)round(pixelPt[0]),(int)round(pixelPt[1])) = 255;
      pseudoImgIdx.at<ushort>((int)round(pixelPt[0]),(int)round(pixelPt[1])) = j;
    }
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(pseudoImg,lines,0.5,CV_PI/180/4,3,5,20);

    for (size_t j = 0; j < lines.size(); j++) {
      cv::Vec4i l = lines[j];
      ushort pt1Idx = pseudoImgIdx.at<ushort>(l[1], l[0]);
      ushort pt2Idx = pseudoImgIdx.at<ushort>(l[3], l[2]);
      Eigen::Vector3d pt1 = sectors[i][pt1Idx].XYZ;
      Eigen::Vector3d pt2 = sectors[i][pt2Idx].XYZ;
      Eigen::Vector3d dir = pt2-pt1;
      dir.normalized();
      std::vector<int> cloudEdgeLineIdx;
      std::vector<int> thetas;
      for (size_t k = 0; k < cloudInEigen.size(); k++)
      {
        if(dir.cross(cloudInEigen[k]-pt1).norm()<lineDis)
        {
          cloudEdgeLineIdx.push_back(k);
          int theta = (int)round(asin(cloudInEigen[k][2]/cloudInEigen[k].norm())*180.0/M_PI);
          auto it = std::find(thetas.begin(),thetas.end(),theta);
          if(it==thetas.end())
            thetas.push_back(theta);
          else{
            cloudEdgeLineIdx.clear();
            break;
          }
        }
      }
      if(cloudEdgeLineIdx.size()<4) continue;
      else
      {
        for(auto x: cloudEdgeLineIdx){
          idxGroupIdxMap[x].push_back(cloudLineGroup.size());
        }
        cloudLineGroup.push_back(cloudEdgeLineIdx);
      }
    }
    cv::Mat cdst;
    cv::cvtColor(pseudoImg, cdst, cv::COLOR_GRAY2BGR);
    for( size_t j = 0; j < lines.size(); j++ )
    {
        cv::Vec4i l = lines[j];
        cv::line( cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 1, cv::LINE_AA);
    }
    cv::imshow("projected",cdst);
    cv::waitKey();
  }
  //remove same points lineing in different line;
  for(auto idxmap: idxGroupIdxMap){
    std::vector<int> groupSize;
    for(int i=0;i<idxmap.second.size();i++){
      groupSize.push_back(cloudLineGroup[idxmap.second[i]].size());
    }
    int maxIdx = *std::max(groupSize.begin(),groupSize.end());
    for(int i=0;i<idxmap.second.size();i++){
      if (i==maxIdx) continue;
      auto it=std::find(cloudLineGroup[idxmap.second[i]].begin(),cloudLineGroup[idxmap.second[i]].end(),idxmap.first);
      cloudLineGroup[idxmap.second[i]].erase(it);
    }
  }
  for (size_t i = 0; i < cloudLineGroup.size(); i++) {
    pcl::PointCloud<PointT> cloudLine;
    if(cloudLineGroup[i].size()<4)
      continue;
    for (size_t j = 0; j < cloudLineGroup[i].size(); j++)
    {
      cloudLine.push_back(cloudIn->points[cloudLineGroup[i][j]]);
      std::sort(cloudLine.begin(),cloudLine.end(),[](PointT pt1,PointT pt2){return pt1.z<pt2.z;});
    }
    cloudOut->push_back(cloudLine);
  }
}
// explicit instantiations