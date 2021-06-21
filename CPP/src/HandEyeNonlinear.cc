#include "HandEyeNonlinear.h"
HandEyeNonlinear::HandEyeNonlinear(/* args */)
{
  qT_ = Eigen::Quaterniond::Identity();
  tT_ = Eigen::Vector3d::Zero();
  scale_ = 1;

}

HandEyeNonlinear::~HandEyeNonlinear()
{
}

void HandEyeNonlinear::SetInitT(Eigen::Quaterniond q,Eigen::Vector3d t){
  qT_ = q;
  tT_ = t;
}
void HandEyeNonlinear::SetInitT(Eigen::Matrix3d R,Eigen::Vector3d t){
 SetInitT(Eigen::Quaterniond(R),t);

}
void HandEyeNonlinear::SetInitT(Eigen::Matrix4d T){
 SetInitT(Eigen::Quaterniond(T.block<3,3>(0,0)),T.block<3,1>(0,3));
}

void HandEyeNonlinear::InputPose(const std::vector<Eigen::Quaterniond> &q1,
                const std::vector<Eigen::Vector3d> &t1,
                const std::vector<Eigen::Quaterniond> &q2,
                const std::vector<Eigen::Vector3d> &t2,
                bool isRelative){
  assert(q1.size()==t1.size());
  assert(q2.size()==t2.size());
  assert(q1.size()==q2.size());
  Eigen::Vector3d qq =  q1[0].vec();
  if(isRelative){
    deltaQ1_ = q1;
    deltaQ2_ = q2;
    deltat1_ = t1;
    deltat2_ = t2;
  }else{
    AbsolutivePose2Relative(q1,t1,&deltaQ1_,&deltat1_);
    AbsolutivePose2Relative(q2,t2,&deltaQ2_,&deltat2_);
  }
}

void HandEyeNonlinear::AbsolutivePose2Relative(
    const std::vector<Eigen::Quaterniond> &q,
    const std::vector<Eigen::Vector3d> &t,
    std::vector<Eigen::Quaterniond> *deltaQ,
    std::vector<Eigen::Vector3d> *deltat) {

  deltaQ->clear();
  deltat->clear();
  for (size_t i = 0; i < q.size() - 1; i++)
  {
    Eigen::Quaterniond qq;
    Eigen::Vector3d tt;
    qq = q[i].conjugate()*q[i+1];
    tt = q[i].toRotationMatrix().transpose()*(t[i+1]-t[i]);
    deltaQ->push_back(qq);
    deltat->push_back(tt);
  }
}

ceres::CostFunction *HandEyeNonlinear::Create(Eigen::Quaterniond q1,
                                              Eigen::Vector3d t1,
                                              Eigen::Quaterniond q2,
                                              Eigen::Vector3d t2,
                                              double dataNum) {
  return (new ceres::AutoDiffCostFunction<CostStruct,1,4,3,1>(
      new CostStruct(q1,t1,q2,t2,dataNum)));
}
void HandEyeNonlinear::run(bool is2DMovement){
  assert(deltaQ1_.size()!=0);
  ceres::Problem problem;
  for (size_t i = 0; i < deltaQ1_.size(); i++) {
  // step 3: get loss function
    ceres::CostFunction *costFunc;
    costFunc = Create(deltaQ1_[i], deltat1_[i],deltaQ2_[i], deltat2_[i],deltaQ1_.size());
    // ceres::LossFunction* lossFunc = new ceres::HuberLoss(1);
    ceres::LossFunction* lossFunc = new ceres::TrivialLoss();
    problem.AddResidualBlock(costFunc, lossFunc, qT_.coeffs().data(),
                             tT_.data(),&scale_);
  }
  std::cout << "Solving nonlinear hand-eye calibration ..." << std::endl;
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.max_num_iterations = 2000;
  // options.line_search_sufficient_function_decrease= 1e-6;
  // options.min_line_search_step_size = 1e-6;
  // options.minimizer_type = ceres::LINE_SEARCH;
  // options.line_search_direction_type = ceres::BFGS;

  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << std::endl;

  if(!is2DMovement){
    return;
  }
  Eigen::Vector3d groundNomal(0,0,0);
  Eigen::Vector3d n1Dir;
  for (size_t i = 0; i < deltaQ1_.size(); i++)
  {
    double theta = 2* acos(deltaQ1_[i].w());
    Eigen::Vector3d n1 = deltaQ1_[i].vec()/sin(theta/2);
    n1.normalize();
    if(i==0){
      n1Dir = n1;
    }else
    {
      n1 = n1Dir.dot(n1)<0?-n1:n1;
    }
    groundNomal = groundNomal+n1;
  }
  groundNomal.normalize();
  tT_ = tT_ - groundNomal.dot(tT_)*groundNomal;
}




Eigen::Quaterniond HandEyeNonlinear::getRotationQ(){
  return qT_;
}
Eigen::Matrix3d HandEyeNonlinear::getRotationM(){
  return qT_.toRotationMatrix();
}
Eigen::Vector3d HandEyeNonlinear::getTranslation(){
  return tT_;
}
Eigen::Matrix4d HandEyeNonlinear::getT(){
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3,3>(0,0) = qT_.toRotationMatrix();
  T.block<3,1>(0,3) = tT_;
  return T;
}

double HandEyeNonlinear::getScale(){
  return scale_;
}
