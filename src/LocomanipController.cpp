#include <mc_tasks/ImpedanceTask.h>
#include <mc_tasks/MetaTaskLoader.h>

#include <BaselineWalkingController/FootManager.h>

#include <LocomanipController/LocomanipController.h>
#include <LocomanipController/ManipManager.h>
#include <LocomanipController/centroidal/CentroidalManagerPreviewControlExtZmp.h>

using namespace LMC;

LocomanipController::LocomanipController(mc_rbdyn::RobotModulePtr rm,
                                         double dt,
                                         const mc_rtc::Configuration & _config,
                                         bool allowEmptyManager)
: BWC::BaselineWalkingController(rm, dt, _config, {}, true)
{
  // Setup tasks
  if(config().has("HandTaskList"))
  {
    for(const auto & handTaskConfig : config()("HandTaskList"))
    {
      Hand hand = strToHand(handTaskConfig("hand"));
      handTasks_.emplace(hand,
                         mc_tasks::MetaTaskLoader::load<mc_tasks::force::ImpedanceTask>(solver(), handTaskConfig));
      handTasks_.at(hand)->name("HandTask_" + std::to_string(hand));
    }
  }
  else
  {
    mc_rtc::log::warning("[LocomanipController] HandTaskList configuration is missing.");
  }

  // Setup managers
  if(!centroidalManager_)
  {
    if(config().has("CentroidalManager"))
    {
      std::string centroidalManagerMethod = config()("CentroidalManager")("method", std::string(""));
      if(centroidalManagerMethod == "PreviewControlExtZmp")
      {
        centroidalManager_ =
            std::make_shared<CentroidalManagerPreviewControlExtZmp>(this, config()("CentroidalManager"));
      }
      else
      {
        if(!allowEmptyManager)
        {
          mc_rtc::log::error_and_throw("[LocomanipController] Invalid centroidalManagerMethod: {}.",
                                       centroidalManagerMethod);
        }
      }
    }
    else
    {
      mc_rtc::log::warning("[LocomanipController] CentroidalManager configuration is missing.");
    }
  }
  if(config().has("ManipManager"))
  {
    manipManager_ = std::make_shared<ManipManager>(this, config()("ManipManager"));
  }
  else
  {
    mc_rtc::log::warning("[LocomanipController] ManipManager configuration is missing.");
  }

  if(config().has("DemoFSM"))
  {
    const auto demo = config()("DemoFSM");
    factory_.load(demo("states"));
    executor_.init(*this, demo);
  }
  datastore().make_call("Locomanip::objectReferencePosition", [this]() -> Eigen::Vector3d
                       { return obj().posW().translation(); });
  datastore().make_call("Locomanip::objectReferenceRpy", [this]() -> Eigen::Vector3d
                       { return mc_rbdyn::rpyFromMat(obj().posW().rotation()); });
  datastore().make_call("Locomanip::leftPhase", [this]() -> double
                       { return manipManager_ ? manipManager_->numericPhase(Hand::Left) : 0.0; });
  datastore().make_call("Locomanip::rightPhase", [this]() -> double
                       { return manipManager_ ? manipManager_->numericPhase(Hand::Right) : 0.0; });
  const auto completion = config()("CompletionState", std::string{});
  datastore().make_call("Locomanip::complete", [this, completion]() -> double
                       { return !completion.empty() && executor_.state() == completion ? 1.0 : 0.0; });

  mc_rtc::log::success("[LocomanipController] Constructed.");
}

void LocomanipController::reset(const mc_control::ControllerResetData & resetData)
{
  BaselineWalkingController::reset(resetData);

  mc_rtc::log::success("[LocomanipController] Reset.");
}

bool LocomanipController::run()
{
  if(exitController_)
  {
    return false;
  }

  t_ += dt();

  if(enableManagerUpdate_)
  {
    // Update managers
    footManager_->update();
    manipManager_->update();
    centroidalManager_->update();
  }

  return mc_control::fsm::Controller::run();
}

void LocomanipController::stop()
{
  // Clean up tasks
  for(const auto & hand : Hands::Both)
  {
    solver().removeTask(handTasks_.at(hand));
  }

  // Clean up managers
  manipManager_->stop();

  BaselineWalkingController::stop();
}
