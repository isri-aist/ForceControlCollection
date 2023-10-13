#include <ForceColl/WrenchDistribution.h>

// std::accumulate
#include <numeric>

using namespace ForceColl;

void WrenchDistribution::Configuration::load(const mc_rtc::Configuration & mcRtcConfig)
{
  mcRtcConfig("wrenchWeight", wrenchWeight);
  mcRtcConfig("regularWeight", regularWeight);
  mcRtcConfig("ridgeForceMinMax", ridgeForceMinMax);
}

WrenchDistribution::WrenchDistribution(const std::vector<std::shared_ptr<Contact>> & contactList,
                                       const mc_rtc::Configuration & mcRtcConfig)
: contactList_(contactList)
{
  config_.load(mcRtcConfig);

  int ridgeNum = 0;
  for(const auto & contact : contactList_)
  {
    ridgeNum += contact->ridgeNum();
  }
  resultWrenchRatio_ = Eigen::VectorXd::Zero(ridgeNum);

  QpSolverCollection::QpSolverType qpSolverType = QpSolverCollection::QpSolverType::Any;
  if(mcRtcConfig.has("qpSolverType"))
  {
    qpSolverType = QpSolverCollection::strToQpSolverType(mcRtcConfig("qpSolverType"));
  }
  qpSolver_ = QpSolverCollection::allocateQpSolver(qpSolverType);
}

sva::ForceVecd WrenchDistribution::run(const sva::ForceVecd & desiredTotalWrench, const Eigen::Vector3d & momentOrigin)
{
  desiredTotalWrench_ = desiredTotalWrench;

  // Return if variable dimension is zero
  if(resultWrenchRatio_.size() == 0)
  {
    resultTotalWrench_ = sva::ForceVecd::Zero();
    return resultTotalWrench_;
  }

  // Resize QP if needed
  {
    int varDim = static_cast<int>(resultWrenchRatio_.size());
    int nIneq = std::accumulate(contactList_.begin(), contactList_.end(), 0,
                                [](int ineq, const auto & contact) { return ineq + (contact->maxWrench_ ? 12 : 0); });
    if(qpCoeff_.dim_var_ != varDim || qpCoeff_.dim_ineq_ != nIneq)
    {
      qpCoeff_.setup(varDim, 0, nIneq);
    }
    if(qpCoeff_.dim_ineq_ != 0)
    {
      qpCoeff_.ineq_mat_.setZero();
    }
  }

  // Construct totalGraspMat
  Eigen::Matrix<double, 6, Eigen::Dynamic> totalGraspMat(6, resultWrenchRatio_.size());
  {
    int ridgeNum = 0;
    int ineqRow = 0;
    for(const auto & contact : contactList_)
    {
      totalGraspMat.middleCols(ridgeNum, contact->ridgeNum()) = contact->graspMat_;
      if(contact->maxWrench_)
      {
        const auto & maxWrench = contact->maxWrench_->vector();
        qpCoeff_.ineq_mat_.block(ineqRow, ridgeNum, 6, contact->ridgeNum()).noalias() = -contact->localGraspMat_;
        qpCoeff_.ineq_mat_.block(ineqRow + 6, ridgeNum, 6, contact->ridgeNum()).noalias() = contact->localGraspMat_;
        qpCoeff_.ineq_vec_.segment(ineqRow, 6) = maxWrench;
        qpCoeff_.ineq_vec_.segment(ineqRow + 6, 6) = maxWrench;
        ineqRow += 12;
      }
      ridgeNum += contact->ridgeNum();
    }
    if(momentOrigin.norm() > 0)
    {
      for(int i = 0; i < ridgeNum; i++)
      {
        // totalGraspMat.col(i).tail<3>() is the force ridge
        totalGraspMat.col(i).head<3>() -= momentOrigin.cross(totalGraspMat.col(i).tail<3>());
      }
    }
  }

  // Solve QP
  {
    Eigen::MatrixXd weightMat = config_.wrenchWeight.vector().asDiagonal();
    qpCoeff_.obj_mat_.noalias() = totalGraspMat.transpose() * weightMat * totalGraspMat;
    qpCoeff_.obj_mat_.diagonal().array() += config_.regularWeight;
    qpCoeff_.obj_vec_.noalias() = -1 * totalGraspMat.transpose() * weightMat * desiredTotalWrench_.vector();
    qpCoeff_.x_min_.setConstant(qpCoeff_.dim_var_, config_.ridgeForceMinMax.first);
    qpCoeff_.x_max_.setConstant(qpCoeff_.dim_var_, config_.ridgeForceMinMax.second);
    resultWrenchRatio_ = qpSolver_->solve(qpCoeff_);
  }

  resultTotalWrench_ = sva::ForceVecd(totalGraspMat * resultWrenchRatio_);

  return resultTotalWrench_;
}

void WrenchDistribution::addToGUI(mc_rtc::gui::StateBuilder & gui,
                                  const std::vector<std::string> & category,
                                  double forceScale,
                                  double fricPyramidScale)
{
  int wrenchRatioIdx = 0;

  for(const auto & contact : contactList_)
  {
    contact->addToGUI(gui, category, forceScale, fricPyramidScale,
                      resultWrenchRatio_.segment(wrenchRatioIdx, contact->ridgeNum()));
    wrenchRatioIdx += contact->ridgeNum();
  }
}
