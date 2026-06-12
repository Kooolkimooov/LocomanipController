#include <LocomanipController/LocomanipController.h>
#include <LocomanipController/states/ExitState.h>

using namespace LMC;

void ExitState::start(mc_control::fsm::Controller & _ctl)
{
    exit(0);
}

bool ExitState::run(mc_control::fsm::Controller &)
{
  return true;
}

void ExitState::teardown(mc_control::fsm::Controller &) {}

EXPORT_SINGLE_STATE("LMC::Exit", ExitState)
