#include <mc_rtc/logging.h>
#include <mc_tasks/ImpedanceTask.h>
#include <BaselineWalkingController/CentroidalManager.h>
#include <BaselineWalkingController/FootManager.h>
#include <Eigen/src/Core/Matrix.h>
#include <LocomanipController/HandTypes.h>
#include <LocomanipController/LocomanipController.h>
#include <LocomanipController/ManipManager.h>
#include <LocomanipController/ManipPhase.h>
#include <LocomanipController/MathUtils.h>
#include <LocomanipController/states/ConfigManipState.h>

using namespace LMC;

void ConfigManipState::sharedPush()
{
  // Both hands push the same object in the same world direction, so estimating
  // per hand lets the two references drift apart and fight each other. Average
  // the measurement in world -- the hand frames are mirrored, so averaging in
  // either frame would cancel the push -- filter once, and split it evenly.
  Eigen::Vector3d measured = Eigen::Vector3d::Zero();
  for(const auto & hand : Hands::Both)
  {
    const auto & task = ctl().handTasks_.at(hand);
    measured += task->surfacePose().rotation().transpose() * task->measuredWrench().force();
  }
  measured *= 0.5;

  if(!sharedForceInit_)
  {
    sharedForce_ = measured;
    sharedForceInit_ = true;
  }
  sharedForce_ = adaptAlpha_ * measured + (1.0 - adaptAlpha_) * sharedForce_;

  for(const auto & hand : Hands::Both)
  {
    const auto & task = ctl().handTasks_.at(hand);
    auto wrench = task->targetWrench();
    wrench.force() = adaptForceProj_.cwiseProduct(task->surfacePose().rotation() * sharedForce_);
    wrench.couple() = adaptMomentProj_.cwiseProduct(
        adaptAlpha_ * task->measuredWrench().couple() + (1.0 - adaptAlpha_) * wrench.couple());

    task->targetWrench(wrench);
    ctl().manipManager_->setRefHandWrench(hand, wrench, ctl().t(), ctl().dt());
  }
}

void ConfigManipState::start(mc_control::fsm::Controller & _ctl)
{
  State::start(_ctl);

  phase_ = 0;

  // Defaults reproduce the hardcoded behaviour this used to have.
  if(config_.has("configs") && config_("configs").has("HandWrenchAdaptation"))
  {
    const auto adaptation = config_("configs")("HandWrenchAdaptation");
    adaptation("enable", adaptHandWrench_);
    adaptation("alpha", adaptAlpha_);
    adaptation("forceProjection", adaptForceProj_);
    adaptation("momentProjection", adaptMomentProj_);
    adaptation("shared", adaptShared_);
    adaptation("worldProjection", adaptWorldProj_);
  }

  output("OK");
}

bool ConfigManipState::run(mc_control::fsm::Controller &)
{
  if(phase_ == 0)
  {
    if(config_.has("configs") && config_("configs")("preUpdateObj", false))
    {
      ctl().manipManager_->appendWaypoint(
          Waypoint(ctl().t(), ctl().t() + 1.0, ctl().manipManager_->objPoseOffset().inv() * ctl().realObj().posW()));
      phase_ = 1;
    }
    else
    {
      phase_ = 2;
    }
  }
  else if(phase_ == 1)
  {
    if(ctl().manipManager_->waypointQueue().empty())
    {
      phase_ = 2;
    }
  }
  else if(phase_ == 2)
  {
    if(config_.has("configs") && config_("configs")("preWalk", false))
    {
      auto convertTo2d = [](const sva::PTransformd & pose) -> Eigen::Vector3d
      {
        return Eigen::Vector3d(pose.translation().x(), pose.translation().y(),
                               mc_rbdyn::rpyFromMat(pose.rotation()).z());
      };
      const sva::PTransformd & initialFootMidpose = projGround(sva::interpolate(
          ctl().footManager_->targetFootPose(Foot::Left), ctl().footManager_->targetFootPose(Foot::Right), 0.5));
      sva::PTransformd objToFootMidTrans =
          config_("configs")("objToFootMidTrans", ctl().manipManager_->config().objToFootMidTrans);
      ctl().footManager_->walkToRelativePose(
          convertTo2d(objToFootMidTrans * ctl().manipManager_->calcRefObjPose(ctl().t()) * initialFootMidpose.inv()));
      phase_ = 3;
    }
    else
    {
      phase_ = 4;
    }
  }
  else if(phase_ == 3)
  {
    if(ctl().footManager_->footstepQueue().empty())
    {
      phase_ = 4;
    }
  }
  else if(phase_ == 4)
  {
    bool isReached = (ctl().manipManager_->manipPhase(Hand::Left)->label() == ManipPhaseLabel::Hold
                      || ctl().manipManager_->manipPhase(Hand::Right)->label() == ManipPhaseLabel::Hold);
    if(config_.has("configs") && config_("configs")("reach", !isReached))
    {
      ctl().manipManager_->reachHandToObj();
      phase_ = 5;
    }
    else
    {
      phase_ = 6;
    }
  }
  else if(phase_ == 5)
  {
    if(ctl().manipManager_->manipPhase(Hand::Left)->label() == ManipPhaseLabel::Hold
       && ctl().manipManager_->manipPhase(Hand::Right)->label() == ManipPhaseLabel::Hold)
    {
      phase_ = 6;
    }
  }
  else if(phase_ == 6)
  {
    if(config_.has("configs") && config_("configs").has("preHandWrenches"))
    {
      for(const auto & handWrenchConfigKV :
          static_cast<std::map<std::string, sva::ForceVecd>>(config_("configs")("preHandWrenches")))
      {
        ctl().manipManager_->setRefHandWrench(strToHand(handWrenchConfigKV.first), handWrenchConfigKV.second,
                                              ctl().t() + 1.0, 1.0);
      }
      phase_ = 7;
    }
    else
    {
      phase_ = 8;
    }
  }
  else if(phase_ == 7)
  {
    if(!ctl().manipManager_->interpolatingRefHandWrench())
    {
      phase_ = 8;
    }
  }
  else if(phase_ == 8)
  {
    if(config_.has("configs") && config_("configs").has("preObjPoseOffset"))
    {
      ctl().manipManager_->setObjPoseOffset(config_("configs")("preObjPoseOffset"), 1.0);
      phase_ = 9;
    }
    else
    {
      phase_ = 10;
    }
  }
  else if(phase_ == 9)
  {
    if(!ctl().manipManager_->interpolatingObjPoseOffset())
    {
      phase_ = 10;
    }
  }
  else if(phase_ == 10)
  {
    if(config_.has("configs") && config_("configs").has("waypointList"))
    {
      double startTime = ctl().t();
      sva::PTransformd pose = ctl().manipManager_->calcRefObjPose(ctl().t());

      for(const auto & waypointConfig : config_("configs")("waypointList"))
      {
        if(waypointConfig.has("startTime"))
        {
          startTime = ctl().t() + static_cast<double>(waypointConfig("startTime"));
        }
        double endTime = startTime;
        if(waypointConfig.has("endTime"))
        {
          endTime = ctl().t() + static_cast<double>(waypointConfig("endTime"));
        }
        else if(waypointConfig.has("duration"))
        {
          endTime = startTime + static_cast<double>(waypointConfig("duration"));
        }
        if(waypointConfig.has("pose"))
        {
          pose = waypointConfig("pose");
        }
        else if(waypointConfig.has("relPose"))
        {
          pose = static_cast<sva::PTransformd>(waypointConfig("relPose")) * pose;
        }
        ctl().manipManager_->appendWaypoint(
            Waypoint(startTime, endTime, pose, waypointConfig("config", mc_rtc::Configuration())));

        startTime = endTime;
      }

      if(config_("configs")("footstep", true))
      {
        ctl().manipManager_->requireFootstepFollowingObj();
      }

      phase_ = 11;
    }
    else if(config_.has("configs") && config_("configs").has("velocityMode"))
    {
      ctl().manipManager_->startVelMode();
      ctl().manipManager_->setRelativeVel(config_("configs")("velocityMode")("velocity"));
      velModeEndTime_ = ctl().t() + static_cast<double>(config_("configs")("velocityMode")("duration"));

      phase_ = 11;
    }
    else
    {
      phase_ = 12;
    }
  }
  else if(phase_ == 11)
  {
    if(adaptHandWrench_ && adaptShared_)
    {
      sharedPush();
    }
    else if(adaptHandWrench_)
    {
    double alpha = adaptAlpha_;

    Eigen::Vector3d force_projection = adaptForceProj_;
    Eigen::Vector3d moment_projection = adaptMomentProj_;

    auto rh_measured_wrench = ctl().handTasks_.at(Hand::Right)->measuredWrench();
    auto lh_measured_wrench = ctl().handTasks_.at(Hand::Left)->measuredWrench();

    auto rh_expected_wrench = ctl().handTasks_.at(Hand::Right)->targetWrench();
    auto lh_expected_wrench = ctl().handTasks_.at(Hand::Left)->targetWrench();

    auto rh_filtered_wrench = alpha * rh_measured_wrench + (1 - alpha) * rh_expected_wrench;
    auto lh_filtered_wrench = alpha * lh_measured_wrench + (1 - alpha) * lh_expected_wrench;

    // The hand frames are robot-specific and mirrored, so a projection meant to
    // keep a world direction -- the horizontal plane, say -- has to be applied there.
    auto project = [&](const Hand & hand, sva::ForceVecd wrench) {
      if(!adaptWorldProj_)
      {
        wrench.force() = force_projection.cwiseProduct(wrench.force());
        wrench.couple() = moment_projection.cwiseProduct(wrench.couple());
        return wrench;
      }
      const Eigen::Matrix3d & E = ctl().handTasks_.at(hand)->surfacePose().rotation();
      wrench.force() = E * force_projection.cwiseProduct(E.transpose() * wrench.force());
      wrench.couple() = E * moment_projection.cwiseProduct(E.transpose() * wrench.couple());
      return wrench;
    };

    auto rh_target_wrench = project(Hand::Right, rh_filtered_wrench);
    auto lh_target_wrench = project(Hand::Left, lh_filtered_wrench);

    ctl().handTasks_.at(Hand::Right)->targetWrench(rh_target_wrench);
    ctl().handTasks_.at(Hand::Left)->targetWrench(lh_target_wrench);

    ctl().manipManager_->setRefHandWrench(Hand::Right, rh_target_wrench, ctl().t(), ctl().dt());
    ctl().manipManager_->setRefHandWrench(Hand::Left, lh_target_wrench, ctl().t(), ctl().dt());
    }

    if(config_.has("configs") && config_("configs").has("CentroidalManager"))
    {
      ctl().centroidalManager_->config().load(config_("configs")("CentroidalManager"));
    }

    if(config_.has("configs") && config_("configs").has("velocityMode"))
    {
      if(ctl().t() > velModeEndTime_ - 1.0 && ctl().manipManager_->velModeEnabled())
      {
        ctl().manipManager_->setRelativeVel(Eigen::Vector3d::Zero());
      }
      if(ctl().t() > velModeEndTime_ && ctl().manipManager_->velModeEnabled())
      {
        ctl().manipManager_->endVelMode();
      }
    }

    if(ctl().manipManager_->waypointQueue().empty() && ctl().footManager_->footstepQueue().empty()
       && !ctl().manipManager_->velModeEnabled())
    {
      phase_ = 12;
    }
  }
  else if(phase_ == 12)
  {
    if(config_.has("configs") && config_("configs").has("postObjPoseOffset"))
    {
      ctl().manipManager_->setObjPoseOffset(config_("configs")("postObjPoseOffset"), 1.0);
      phase_ = 13;
    }
    else
    {
      phase_ = 14;
    }
  }
  else if(phase_ == 13)
  {
    if(!ctl().manipManager_->interpolatingObjPoseOffset())
    {
      phase_ = 14;
    }
  }
  else if(phase_ == 14)
  {
    if(config_.has("configs") && config_("configs").has("postHandWrenches"))
    {
      for(const auto & handWrenchConfigKV :
          static_cast<std::map<std::string, sva::ForceVecd>>(config_("configs")("postHandWrenches")))
      {
        ctl().manipManager_->setRefHandWrench(strToHand(handWrenchConfigKV.first), handWrenchConfigKV.second,
                                              ctl().t() + 1.0, 1.0);
      }
      phase_ = 15;
    }
    else
    {
      phase_ = 16;
    }
  }
  else if(phase_ == 15)
  {
    if(!ctl().manipManager_->interpolatingRefHandWrench())
    {
      phase_ = 16;
    }
  }
  else if(phase_ == 16)
  {
    if(config_.has("configs") && config_("configs")("release", true))
    {
      ctl().manipManager_->releaseHandFromObj();
      phase_ = 17;
    }
    else
    {
      phase_ = 18;
    }
  }
  else if(phase_ == 17)
  {
    if(ctl().manipManager_->manipPhase(Hand::Left)->label() == ManipPhaseLabel::Free
       && ctl().manipManager_->manipPhase(Hand::Right)->label() == ManipPhaseLabel::Free)
    {
      phase_ = 18;
    }
  }

  return phase_ == 18;
}

void ConfigManipState::teardown(mc_control::fsm::Controller &)
{
  if(config_.has("configs") && config_("configs").has("CentroidalManager"))
  {
    ctl().centroidalManager_->config().load(ctl().config()("CentroidalManager"));
  }
}

EXPORT_SINGLE_STATE("LMC::ConfigManip", ConfigManipState)
