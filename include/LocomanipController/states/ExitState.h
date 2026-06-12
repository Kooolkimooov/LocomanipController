#pragma once

#include <LocomanipController/State.h>

namespace LMC
{
/** \brief FSM state that exits the controller. */
struct ExitState : State
{
public:
  /** \brief Start. */
  void start(mc_control::fsm::Controller & ctl) override;

  /** \brief Run. */
  bool run(mc_control::fsm::Controller & ctl) override;

  /** \brief Teardown. */
  void teardown(mc_control::fsm::Controller & ctl) override;
};
} // namespace LMC
