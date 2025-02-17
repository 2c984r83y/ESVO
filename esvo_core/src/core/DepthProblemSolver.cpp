#include <esvo_core/core/DepthProblemSolver.h>

#include <thread>
#include <functional>
#include <fstream>
#include <esvo_core/tools/TicToc.h>
//#define DEPTH_PROBLEM_SOLVER_LOG
namespace esvo_core
{
namespace core
{
DepthProblemSolver::DepthProblemSolver(
  CameraSystem::Ptr & camSysPtr,
  std::shared_ptr<DepthProblemConfig> & dpConfigPtr,
  DepthProblemType dpType,
  size_t numThread )
  :
  camSysPtr_(camSysPtr),
  dpConfigPtr_(dpConfigPtr),
  dpType_(dpType),
  NUM_THREAD_(numThread)
{}

DepthProblemSolver::~DepthProblemSolver()
{
}

// dpSolver_.solve(&vEMP, &TS_obs_, vdp);
void DepthProblemSolver::solve(
  // Block Match result
  std::vector<EventMatchPair>* pvEMP, 
  StampedTimeSurfaceObs* pStampedTsObs,
  // Nonlinear Optimization result
  std::vector<DepthPoint> &vdp )
{
//  TicToc tt;
//  tt.tic();
  // distribute the loads
  std::vector<Job> jobs(NUM_THREAD_);
  for(size_t i = 0;i < NUM_THREAD_;i++)
  {
    jobs[i].i_thread_ = i;
    jobs[i].pvEMP_ = pvEMP;
    jobs[i].pStamped_TS_obs_ = pStampedTsObs;
    if(dpType_ == NUMERICAL)
      jobs[i].numDiff_dProblemPtr_ = std::make_shared<Eigen::NumericalDiff<DepthProblem> >(dpConfigPtr_, camSysPtr_);
    else if(dpType_ == ANALYTICAL)
      jobs[i].dProblemPtr_ = std::make_shared<DepthProblem>(dpConfigPtr_, camSysPtr_); //NOTE: The ANALYTICAL version is not provided in this version.
    else
    {
      LOG(ERROR) << "Wrong Depth Problem Type is assigned!!!";
      exit(-1);
    }
    jobs[i].vdpPtr_ = std::make_shared<std::vector<DepthPoint> >();
  }
//  LOG(INFO) << "(DepthProblemSolver) distribute the loads: " << tt.toc() << " ms.";
//  tt.tic();
  // 
  std::vector<std::thread> threads;
  for(size_t i = 0; i < NUM_THREAD_;i++)
    threads.emplace_back(std::bind(&DepthProblemSolver::solve_multiple_problems, this, jobs[i]));
  for( auto & thread : threads)
  {
    if(thread.joinable())
      thread.join();
  }
  //
  vdp.clear();
  size_t numPoints = 0;
  for(size_t i = 0;i < NUM_THREAD_;i++)
  {
#ifdef DEPTH_PROBLEM_SOLVER_LOG
    LOG(INFO) << "The " << i << " thread reconstructs " << jobs[i].vdpPtr_->size() << " points";
#endif
    numPoints += jobs[i].vdpPtr_->size();
  }
  vdp.reserve(numPoints);
  for(size_t i = 0;i < NUM_THREAD_;i++)
    vdp.insert(vdp.end(), jobs[i].vdpPtr_->begin(), jobs[i].vdpPtr_->end());
//  LOG(INFO) << "(DepthProblemSolver) copy results: " << tt.toc() << " ms.";
}

void DepthProblemSolver::solve_multiple_problems(Job & job)
{
  size_t i_thread = job.i_thread_;
  size_t numEvent = job.pvEMP_->size();
  job.vdpPtr_->clear();
  job.vdpPtr_->reserve(numEvent / NUM_THREAD_ + 1);

  StampedTimeSurfaceObs* pStampedTsObs = job.pStamped_TS_obs_;

  // loop through vdp and call solve_single_problem
  for(size_t i = i_thread; i < numEvent; i+=NUM_THREAD_)
  {
    // 得到坐标、位姿变换矩阵、初始深度估计
    Eigen::Vector2d coor = (*job.pvEMP_)[i].x_left_;
    Eigen::Matrix<double, 4, 4> T_world_virtual = (*job.pvEMP_)[i].trans_.getTransformationMatrix();
    double d_init = (*job.pvEMP_)[i].invDepth_;

    double result[3];
    bool bProblemSolved = false;
    if(dpType_ == NUMERICAL)
    {
      // setProblem 坐标、位姿变换矩阵、 TimeSurface
      job.numDiff_dProblemPtr_->setProblem(coor, T_world_virtual, pStampedTsObs);
      bProblemSolved = solve_single_problem_numerical(d_init,job.numDiff_dProblemPtr_, result);
    }
    else if(dpType_ == ANALYTICAL)
    {
      LOG(ERROR) << "Not supported in this release!!! Please use NUMERICAL.";
      exit(-1);
    }
    else
    {
      LOG(ERROR) << "Wrong Depth Problem Type is assigned!!!";
      exit(-1);
    }

    if(bProblemSolved)
    {
      DepthPoint dp(std::floor(coor(1)), std::floor(coor(0)));
      dp.update_x(coor);
      Eigen::Vector3d p_cam;
      camSysPtr_->cam_left_ptr_->cam2World(coor, result[0], p_cam);
      dp.update_p_cam(p_cam);
      if(strcmp(dpConfigPtr_->LSnorm_.c_str(), "l2") == 0)
        dp.update(result[0], result[1]);
      else if(strcmp(dpConfigPtr_->LSnorm_.c_str(), "Tdist") == 0)
      {
        double scale2_rho = result[1] * (dpConfigPtr_->td_nu_ - 2) / dpConfigPtr_->td_nu_;
        dp.update_studentT(result[0], scale2_rho, result[1], dpConfigPtr_->td_nu_);
      }
      else
        exit(-1);

      dp.residual() = result[2];
      dp.updatePose(T_world_virtual);
      job.vdpPtr_->push_back(dp);
    }
  }
}

bool DepthProblemSolver::solve_single_problem_numerical(
  double d_init, // 初始深度估计
  std::shared_ptr< Eigen::NumericalDiff<DepthProblem> > & dProblemPtr, // 数值微分对象指针
  double* result) // 存储优化结果的数组指针
{
  // 创建具有一个元素的 Eigen 向量 x，表示初始深度估计
  // 大小为 1
  Eigen::VectorXd x(1); 
  // << 是 Eigen 向量的赋值运算符
  x << d_init;

  // 创建 Levenberg-Marquardt 优化器 lm，使用数值微分对象进行初始化
  // 要优化的目标函数是 DepthProblem 类中的 operator() 函数，
  // 它计算了当前深度估计下的重投影误差。
  // 这个函数的输入是一个深度值，输出是重投影误差。
  // 优化器的目标是最小化这个函数的输出，以得到最优的深度估计。
  Eigen::LevenbergMarquardt<Eigen::NumericalDiff<DepthProblem>, double> lm(*(dProblemPtr.get()));
  lm.resetParameters();
  lm.parameters.ftol = 1e-6; // 设置函数值变化的容忍度
  lm.parameters.xtol = 1e-6; // 设置参数变化的容忍度
  lm.parameters.maxfev = dpConfigPtr_->MAX_ITERATION_ * 3; // 设置最大迭代次数

  // 使用初始深度估计初始化优化器
  if(lm.minimizeInit(x) == Eigen::LevenbergMarquardtSpace::ImproperInputParameters)
  {
    LOG(ERROR) << "ImproperInputParameters for LM (Mapping)." << std::endl;
    return false;
  }

  size_t iteration = 0;
  int optimizationState = 0;

  // 运行优化器，最大迭代次数由 dpConfigPtr_->MAX_ITERATION_ 参数指定
  while(true)
  {
    Eigen::LevenbergMarquardtSpace::Status status = lm.minimizeOneStep(x); // 执行一次优化步骤

    iteration++;
    if(iteration >= dpConfigPtr_->MAX_ITERATION_)
      break;

    bool terminate = false;
    if(status == 2 || status == 3) // 判断优化状态
    {
      switch (optimizationState)
      {
        case 0:
        {
          optimizationState++;
          break;
        }
        case 1:
        {
          terminate = true;
          break;
        }
      }
    }
    if(terminate)
      break;
  }

  // 由于 Eigen 中没有设置参数边界的方法，因此在此应用方便的异常值拒绝方法
  if(x(0) <= 0.001) // 如果深度估计小于等于 0.001，则返回 false
    return false;

  // 更新结果数组
  result[0] = x(0);

  // 计算方差
  Eigen::internal::covar(lm.fjac, lm.permutation.indices());
  if(dpConfigPtr_->LSnorm_ == "l2") // 如果使用 L2 范数
  {
    double fnorm = lm.fvec.blueNorm();
    double covfac = fnorm * fnorm / (dProblemPtr->values() - dProblemPtr->inputs());
    Eigen::MatrixXd cov = covfac * lm.fjac.topLeftCorner<1,1>();
    result[1] = cov(0,0);
  }
  if(dpConfigPtr_->LSnorm_ == "Tdist") // 如果使用 T 分布
  {
    Eigen::MatrixXd invSumJtT = lm.fjac.topLeftCorner<1,1>();
    result[1] = std::pow(dpConfigPtr_->td_stdvar_,2) * invSumJtT(0,0);
  }
  result[2] = lm.fnorm * lm.fnorm; // 计算残差平方和
  return true;
}

// 移除无效的深度点
void
DepthProblemSolver::pointCulling(
  std::vector<DepthPoint> &vdp,
  double std_variance_threshold,
  double cost_threshold,
  double invDepth_min_range,
  double invDepth_max_range)
{
  std::vector<DepthPoint> vdp_culled;
  vdp_culled.reserve(vdp.size());
  std::vector<double> vDepth;
  vDepth.reserve(10000);
  for(size_t i = 0; i < vdp.size();i++)
  {
    if(vdp[i].variance() <= pow(std_variance_threshold,2) &&
      vdp[i].residual() <= cost_threshold &&
      vdp[i].valid() &&
      vdp[i].invDepth() >= invDepth_min_range &&
      vdp[i].invDepth() <= invDepth_max_range)
    {
      vdp_culled.push_back(vdp[i]);
      vDepth.emplace_back(1.0 / vdp[i].invDepth());
    }
  }
  vdp = vdp_culled;
#ifdef DEPTH_PROBLEM_SOLVER_LOG
  LOG(INFO) << "(culling) max depth: " << *std::max_element(vDepth.begin(), vDepth.end());
#endif
}

DepthProblemType DepthProblemSolver::getProblemType()
{
  return dpType_;
}

}//end of namespace core
}//end of namespace esvo_core

