#pragma once

#include <string>
#include <unordered_map>

#include <LocomanipController/State.h>

namespace LMC
{
/** \brief FSM state to initialize. */
struct InitialState : State
{
public:
  /** \brief Start. */
  void start(mc_control::fsm::Controller & ctl) override;

  /** \brief Run. */
  bool run(mc_control::fsm::Controller & ctl) override;

  /** \brief Teardown. */
  void teardown(mc_control::fsm::Controller & ctl) override;

protected:
  /** \brief Check whether state is completed. */
  bool complete() const;

  /** \brief Re-send gripper commands that the gripper safety cut short. */
  void retryGripperCommands();

protected:
  //! Phase
  int phase_ = 0;

  //! Function to interpolate task stiffness
  std::shared_ptr<TrajColl::CubicInterpolator<double>> stiffnessRatioFunc_;

  //! Stiffness of CoM task
  Eigen::Vector3d comTaskStiffness_ = Eigen::Vector3d::Zero();

  //! Stiffness of base link orientation task
  Eigen::Vector3d baseOriTaskStiffness_ = Eigen::Vector3d::Zero();

  //! Stiffness of foot tasks
  std::unordered_map<Foot, Eigen::Vector6d> footTasksStiffness_;

  //! Requested opening per gripper, kept to detect a command cut short
  std::unordered_map<std::string, double> gripperOpenings_;

  //! Time until which a cut-short gripper command is re-sent [sec]
  double gripperRetryEndTime_ = 0.0;
};
} // namespace LMC
