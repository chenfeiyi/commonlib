#include <string>
#include <vector>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <ceres/cost_function.h>
#include <ceres/loss_function.h>
struct CostStruct {
  // target_line: 0-2: first point, 3-5: second point;
  CostStruct(Eigen::Quaterniond q1,Eigen::Vector3d t1,Eigen::Quaterniond q2,Eigen::Vector3d t2,double dataNum)
      : q1_(q1),q2_(q2),t1_(t1),t2_(t2),dataNum_(dataNum){}

  template <typename T>  // 自动求导才会用的定义方式
  bool operator()(const T* const qT, const T* const tT,const T* scale,
                  T* residual) const {
    
    T lambda1=(T)1;
    T lambda2=(T)1;
    T lambda3=(T)2e3;
    T dataNum = (T)dataNum_;

    double theta1 = 2*acos(q1_.w());
    Eigen::Vector3d n1 = q1_.vec()/sin(theta1/2);
    double theta2 = 2*acos(q2_.w());
    Eigen::Vector3d n2 = q2_.vec()/sin(theta2/2);
    Eigen::Quaternion<T> q1((T)q1_.w(),(T)q1_.x(),(T)q1_.y(),(T)q1_.z());
    Eigen::Quaternion<T> qSpin1((T)0,(T)n1[0],(T)n1[1],(T)n1[2]);
    Eigen::Quaternion<T> q2((T)q2_.w(),(T)q2_.x(),(T)q2_.y(),(T)q2_.z());
    Eigen::Quaternion<T> qSpin2((T)0,(T)n2[0],(T)n2[1],(T)n2[2]);


    Eigen::Quaternion<T> t1Q((T)0,(T)t1_[0],(T)t1_[1],(T)t1_[2]);
    Eigen::Quaternion<T> t2Q((T)0,(T)t2_[0],(T)t2_[1],(T)t2_[2]);
    
    Eigen::Quaternion<T> qTQ(qT[3],qT[0],qT[1],qT[2]);
    Eigen::Matrix<T,3,1> tTQ(tT[0],tT[1],tT[2]);
    

    Eigen::Matrix<T,3,1> f1 = qSpin1.vec() - (qTQ*qSpin2*qTQ.conjugate()).vec();
    Eigen::Matrix<T,3,1> f2 = (q1.toRotationMatrix()-Eigen::Matrix<T,3,3>::Identity())*tTQ - ((qTQ*t2Q*qTQ.conjugate()).vec() - scale[0]*t1Q.vec());
    residual[0] = lambda1 * f1.norm()+ lambda2 * f2.norm()+
                  lambda3 * ((T)1 - qTQ.coeffs().norm()*qTQ.coeffs().norm())*((T)1 - qTQ.coeffs().norm()*qTQ.coeffs().norm())/dataNum;
    return true;
  }
 private:
 Eigen::Quaterniond q1_;
 Eigen::Vector3d t1_;
 Eigen::Quaterniond q2_;
 Eigen::Vector3d t2_;
 double dataNum_;
};
class HandEyeNonlinear
{
private:
  Eigen::Quaterniond qT_; // rotation from 2 to 1
  Eigen::Vector3d tT_; // translation from 2 to 1
  double scale_;
  std::vector<Eigen::Quaterniond> deltaQ1_;
  std::vector<Eigen::Quaterniond> deltaQ2_;
  std::vector<Eigen::Vector3d> deltat1_;
  std::vector<Eigen::Vector3d> deltat2_;
public:
  HandEyeNonlinear();
  ~HandEyeNonlinear();
  void SetInitT(Eigen::Quaterniond q,Eigen::Vector3d t);
  void SetInitT(Eigen::Matrix3d R,Eigen::Vector3d t);
  void SetInitT(Eigen::Matrix4d T);
  Eigen::Quaterniond getRotationQ();
  Eigen::Matrix3d getRotationM();
  Eigen::Vector3d getTranslation();
  Eigen::Matrix4d getT();
  double getScale();
  void InputPose(const std::vector<Eigen::Quaterniond> &q1,
                 const std::vector<Eigen::Vector3d> &t1,
                 const std::vector<Eigen::Quaterniond> &q2,
                 const std::vector<Eigen::Vector3d> &t2,
                 bool isRelative = false);
  void AbsolutivePose2Relative(const std::vector<Eigen::Quaterniond> &q,
                               const std::vector<Eigen::Vector3d> &t,
                               std::vector<Eigen::Quaterniond> *deltaQ,
                               std::vector<Eigen::Vector3d> *deltat);
  ceres::CostFunction *Create(Eigen::Quaterniond q1, Eigen::Vector3d t1,
                              Eigen::Quaterniond q2, Eigen::Vector3d t2,
                              double dataNum);
  void run(bool is2DMovement=false);
};




