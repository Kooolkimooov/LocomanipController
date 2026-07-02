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

void ConfigManipState::start(mc_control::fsm::Controller & _ctl)
{
  State::start(_ctl);

  phase_ = 0;

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
    // if(config_.has("configs") && !config_("configs").has("preHandWrenches"))
    // {
    double alpha = 0.1;

    Eigen::Matrix3d right_to_left_hand_diff;
    // x and y- are reversed in left compared to right
    right_to_left_hand_diff << -1, 0, 0, 0, -1, 0, 0, 0, 1;
    // std::cout << right_to_left_hand_diff;

    mc_rtc::log::warning("=============================");

    auto rh_measured_force = ctl().handTasks_.at(Hand::Right)->measuredWrench().force();
    auto lh_measured_force = ctl().handTasks_.at(Hand::Left)->measuredWrench().force();

    mc_rtc::log::info("lhm {:+8.3f}, {:+8.3f}, {:+8.3f}", lh_measured_force[0], lh_measured_force[1],
                      lh_measured_force[2]);
    mc_rtc::log::info("rhm {:+8.3f}, {:+8.3f}, {:+8.3f}", rh_measured_force[0], rh_measured_force[1],
                      rh_measured_force[2]);

    auto measured_force = rh_measured_force + right_to_left_hand_diff * lh_measured_force;

    mc_rtc::log::info("m   {:+8.3f}, {:+8.3f}, {:+8.3f}", measured_force[0], measured_force[1], measured_force[2]);

    auto rh_expected_force = ctl().handTasks_.at(Hand::Right)->targetWrench().force();
    auto lh_expected_force = ctl().handTasks_.at(Hand::Left)->targetWrench().force();

    mc_rtc::log::info("lhe {:+8.3f}, {:+8.3f}, {:+8.3f}", lh_expected_force[0], lh_expected_force[1],
                      lh_expected_force[2]);
    mc_rtc::log::info("rhe {:+8.3f}, {:+8.3f}, {:+8.3f}", rh_expected_force[0], rh_expected_force[1],
                      rh_expected_force[2]);

    auto expected_force = rh_expected_force + right_to_left_hand_diff * lh_expected_force;

    mc_rtc::log::info("e   {:+8.3f}, {:+8.3f}, {:+8.3f}", expected_force[0], expected_force[1], expected_force[2]);

    auto filtered_force = alpha * measured_force + (1 - alpha) * expected_force;

    mc_rtc::log::info("f   {:+8.3f}, {:+8.3f}, {:+8.3f}", filtered_force[0], filtered_force[1], filtered_force[2]);

    auto new_expected_force = filtered_force / 2.0;

    mc_rtc::log::info("ne  {:+8.3f}, {:+8.3f}, {:+8.3f}", new_expected_force[0], new_expected_force[1],
                      new_expected_force[2]);

    auto rh_new_expected_force = new_expected_force;
    auto lh_new_expected_force = right_to_left_hand_diff * new_expected_force;

    mc_rtc::log::info("lhe {:+8.3f}, {:+8.3f}, {:+8.3f}", lh_new_expected_force[0], lh_new_expected_force[1],
                      lh_new_expected_force[2]);
    mc_rtc::log::info("rhe {:+8.3f}, {:+8.3f}, {:+8.3f}", rh_new_expected_force[0], rh_new_expected_force[1],
                      rh_new_expected_force[2]);

    auto rh_target_wrench = sva::ForceVecd(Eigen::Vector3d::Zero(), rh_new_expected_force);
    auto lh_target_wrench = sva::ForceVecd(Eigen::Vector3d::Zero(), lh_new_expected_force);

    ctl().handTasks_.at(Hand::Right)->targetWrench(rh_target_wrench);
    ctl().handTasks_.at(Hand::Left)->targetWrench(lh_target_wrench);

    ctl().manipManager_->setRefHandWrench(Hand::Right, rh_target_wrench, ctl().t(), ctl().dt());
    ctl().manipManager_->setRefHandWrench(Hand::Left, lh_target_wrench, ctl().t(), ctl().dt());

    // }

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
