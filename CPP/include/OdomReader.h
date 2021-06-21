#pragma  once
#include <vector>
#include <string>
#include <fstream>
#include <iostream> 
#include <Eigen/Dense>
#include "log.h"
struct OdomData
{
  double stamp;
  Eigen::Vector4d q;
  Eigen::Vector3d t;
};

class OdomReader
{
private:
  /* data */
public:
  OdomReader();
  bool LoadAndSync(std::string imgOdomPath,std::string pcdOdomPath);
  void LoadOdomFile(std::string odomPath,std::vector<OdomData> *odom);
  std::vector<OdomData> GetImgOdom();
  std::vector<OdomData> GetPCDOdom();
  void SetSyncTime(float syncTime);

  ~OdomReader();
private:
float syncTime_;
std::vector<OdomData> imgOdom_;
std::vector<OdomData> pcdOdom_;

};

