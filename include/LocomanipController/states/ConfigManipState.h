#pragma once

#include <Eigen/Core>

#include <LocomanipController/State.h>

namespace LMC
{
/** \brief FSM state to send manipulation commands from configuration. */
struct ConfigManipState : State
{
public:
  /** \brief Start. */
  void start(mc_control::fsm::Controller & ctl) override;

  /** \brief Run. */
  bool run(mc_control::fsm::Controller & ctl) override;

  /** \brief Teardown. */
  void teardown(mc_control::fsm::Controller & ctl) override;

protected:
  /** \brief Blend one shared force estimate into both hands' references. */
  void sharedPush();

  //! Phase
  int phase_ = 0;

  //! End time of velocity mode [sec]
  double velModeEndTime_ = 0.0;

  //! Whether the push phase blends the measured hand wrench into the target
  bool adaptHandWrench_ = true;

  //! Blend coefficient per control cycle; its time constant is dt / alpha
  double adaptAlpha_ = 0.02;

  //! Hand-frame axes the blend is allowed to act on
  Eigen::Vector3d adaptForceProj_ = Eigen::Vector3d(1, 0, 1);
  Eigen::Vector3d adaptMomentProj_ = Eigen::Vector3d(0, 1, 0);

  //! Whether both hands share one force estimate instead of blending their own
  bool adaptShared_ = false;

  //! Shared force estimate, in world [N]
  Eigen::Vector3d sharedForce_ = Eigen::Vector3d::Zero();

  //! Whether the shared estimate has been seeded from the first measurement
  bool sharedForceInit_ = false;
};
} // namespace LMC
