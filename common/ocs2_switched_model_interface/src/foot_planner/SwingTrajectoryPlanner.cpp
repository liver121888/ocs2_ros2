//
// Created by rgrandia on 13.03.20.
//

#include "ocs2_switched_model_interface/foot_planner/SwingTrajectoryPlanner.h"

#include <ocs2_core/misc/Lookup.h>

#include "ocs2_switched_model_interface/core/MotionPhaseDefinition.h"
#include "ocs2_switched_model_interface/core/Rotations.h"

namespace switched_model {

SwingTrajectoryPlanner::SwingTrajectoryPlanner(SwingTrajectoryPlannerSettings settings,
                                               const KinematicsModelBase<scalar_t>& kinematicsModel)
    : settings_(std::move(settings)), kinematicsModel_(kinematicsModel.clone()), terrainModel_(nullptr) {
  setDefaultSwingProfile();
}

void SwingTrajectoryPlanner::updateTerrain(std::unique_ptr<TerrainModel> terrainModel) {
  terrainModel_ = std::move(terrainModel);
}

const SignedDistanceField* SwingTrajectoryPlanner::getSignedDistanceField() const {
  if (terrainModel_) {
    return terrainModel_->getSignedDistanceField();
  } else {
    return nullptr;
  }
}

void SwingTrajectoryPlanner::updateSwingMotions(scalar_t initTime, scalar_t finalTime, const comkino_state_t& currentState,
                                                const ocs2::TargetTrajectories& targetTrajectories,
                                                const feet_array_t<std::vector<ContactTiming>>& contactTimingsPerLeg) {
  if (!terrainModel_) {
    throw std::runtime_error("[SwingTrajectoryPlanner] terrain cannot be null. Update the terrain before planning swing motions");
  }

  // Need a copy to provide joint references later (possibly adapted with inverse kinematics)
  targetTrajectories_ = targetTrajectories;

  const auto basePose = getBasePose(currentState);
  const auto feetPositions = kinematicsModel_->feetPositionsInOriginFrame(basePose, getJointPositions(currentState));

  for (int leg = 0; leg < NUM_CONTACT_POINTS; leg++) {
    const auto& contactTimings = contactTimingsPerLeg[leg];

    // Update last contacts
    if (!contactTimings.empty()) {
      if (startsWithStancePhase(contactTimings)) {
        // If currently in contact -> update expected liftoff.
        if (hasEndTime(contactTimings.front())) {
          updateLastContact(leg, contactTimings.front().end, feetPositions[leg], *terrainModel_);
        } else {  // Expected liftoff unknown, set to end of horizon
          updateLastContact(leg, finalTime, feetPositions[leg], *terrainModel_);
        }
      } else {
        // If currently in swing -> verify that liftoff was before the horizon. If not, assume liftoff happened exactly at initTime
        if (lastContacts_[leg].first > initTime) {
          updateLastContact(leg, initTime, feetPositions[leg], *terrainModel_);
        }
      }
    }

    // Nominal footholds / terrain planes
    std::tie(nominalFootholdsPerLeg_[leg], heuristicFootholdsPerLeg_[leg]) =
        selectNominalFootholdTerrain(leg, contactTimings, targetTrajectories, initTime, currentState, finalTime, *terrainModel_);

    // Create swing trajectories
    std::tie(feetNormalTrajectoriesEvents_[leg], feetNormalTrajectories_[leg]) = generateSwingTrajectories(leg, contactTimings, finalTime);
  }
}

const FootPhase& SwingTrajectoryPlanner::getFootPhase(size_t leg, scalar_t time) const {
  const auto index = ocs2::lookup::findIndexInTimeArray(feetNormalTrajectoriesEvents_[leg], time);
  return *feetNormalTrajectories_[leg][index];
}

joint_coordinate_t SwingTrajectoryPlanner::getJointPositionsReference(scalar_t time) const {
  return getJointPositions(targetTrajectories_.getDesiredState(time));
}

joint_coordinate_t SwingTrajectoryPlanner::getJointVelocitiesReference(scalar_t time) const {
  return getJointVelocities(targetTrajectories_.getDesiredInput(time));
}

auto SwingTrajectoryPlanner::generateSwingTrajectories(int leg, const std::vector<ContactTiming>& contactTimings, scalar_t finalTime) const
    -> std::pair<std::vector<scalar_t>, std::vector<std::unique_ptr<FootPhase>>> {
  std::vector<scalar_t> eventTimes;
  std::vector<std::unique_ptr<FootPhase>> footPhases;

  // First swing phase
  if (startsWithSwingPhase(contactTimings)) {
    SwingPhase::SwingEvent liftOff{lastContacts_[leg].first, settings_.liftOffVelocity, &lastContacts_[leg].second};
    SwingPhase::SwingEvent touchDown = [&] {
      if (touchesDownAtLeastOnce(contactTimings)) {
        return SwingPhase::SwingEvent{contactTimings.front().start, settings_.touchDownVelocity,
                                      &nominalFootholdsPerLeg_[leg].front().plane};
      } else {
        return SwingPhase::SwingEvent{finalTime + settings_.referenceExtensionAfterHorizon, 0.0, nullptr};
      }
    }();

    SwingPhase::SwingProfile swingProfile = defaultSwingProfile_;
    if (settings_.swingTrajectoryFromReference) {
      swingProfile.nodes = extractSwingProfileFromReference(leg, liftOff, touchDown);
    } else {
      applySwingMotionScaling(liftOff, touchDown, swingProfile);
    }

    footPhases.emplace_back(new SwingPhase(liftOff, touchDown, swingProfile, terrainModel_.get(), settings_.errorGain));
  }

  // Loop through contact phases
  for (int i = 0; i < contactTimings.size(); ++i) {
    const auto& currentContactTiming = contactTimings[i];
    const ConvexTerrain& nominalFoothold = nominalFootholdsPerLeg_[leg][i];

    // If phase starts after the horizon, we don't need to plan for it
    if (currentContactTiming.start > finalTime) {
      break;
    }

    // generate contact phase
    if (hasStartTime(currentContactTiming)) {
      eventTimes.push_back(currentContactTiming.start);
    }
    footPhases.emplace_back(new StancePhase(nominalFoothold, settings_.errorGain, settings_.terrainMargin));

    // If contact phase extends beyond the horizon, we can stop planning.
    if (!hasEndTime(currentContactTiming) || currentContactTiming.end > finalTime) {
      break;
    }

    // generate swing phase afterwards
    SwingPhase::SwingEvent liftOff{currentContactTiming.end, settings_.liftOffVelocity, &nominalFoothold.plane};
    SwingPhase::SwingEvent touchDown = [&] {
      const bool nextContactExists = (i + 1) < contactTimings.size();
      if (nextContactExists) {
        return SwingPhase::SwingEvent{contactTimings[i + 1].start, settings_.touchDownVelocity, &nominalFootholdsPerLeg_[leg][i + 1].plane};
      } else {
        return SwingPhase::SwingEvent{finalTime + settings_.referenceExtensionAfterHorizon, 0.0, nullptr};
      }
    }();

    SwingPhase::SwingProfile swingProfile = defaultSwingProfile_;
    if (settings_.swingTrajectoryFromReference) {
      swingProfile.nodes = extractSwingProfileFromReference(leg, liftOff, touchDown);
    } else {
      applySwingMotionScaling(liftOff, touchDown, swingProfile);
    }

    eventTimes.push_back(currentContactTiming.end);
    footPhases.emplace_back(new SwingPhase(liftOff, touchDown, swingProfile, terrainModel_.get(), settings_.errorGain));
  }

  return std::make_pair(eventTimes, std::move(footPhases));
}

void SwingTrajectoryPlanner::applySwingMotionScaling(SwingPhase::SwingEvent& liftOff, SwingPhase::SwingEvent& touchDown,
                                                     SwingPhase::SwingProfile& swingProfile) const {
  const scalar_t scaling = [&]() {
    if (std::isnan(liftOff.time) || std::isnan(touchDown.time)) {
      return 1.0;
    } else {
      return std::min(1.0, (touchDown.time - liftOff.time) / settings_.swingTimeScale);
    }
  }();

  if (scaling < 1.0) {
    liftOff.velocity *= scaling;
    touchDown.velocity *= scaling;
    swingProfile.sdfMidswingMargin = scaling * settings_.sdfMidswingMargin;
    for (auto& node : swingProfile.nodes) {
      node.swingHeight *= scaling;
      node.normalVelocity *= scaling;
    }
  }
}

std::pair<std::vector<ConvexTerrain>, std::vector<vector3_t>> SwingTrajectoryPlanner::selectNominalFootholdTerrain(
    int leg, const std::vector<ContactTiming>& contactTimings, const ocs2::TargetTrajectories& targetTrajectories, scalar_t initTime,
    const comkino_state_t& currentState, scalar_t finalTime, const TerrainModel& terrainModel) const {
  // Zmp preparation
  const auto initBasePose = getBasePose(currentState);
  const auto initBasePosition = getPositionInOrigin(initBasePose);
  const auto initBaseOrientation = getOrientation(initBasePose);
  const auto initBaseTwistInBase = getBaseLocalVelocities(currentState);
  const auto initBaseLinearVelocityInWorld = rotateVectorBaseToOrigin(getLinearVelocity(initBaseTwistInBase), initBaseOrientation);
  vector_t initDesiredState = targetTrajectories.getDesiredState(initTime);
  const base_coordinate_t initDesiredBasePose = initDesiredState.head<BASE_COORDINATE_SIZE>();
  const auto initDesiredBasePosition = getPositionInOrigin(initDesiredBasePose);
  const base_coordinate_t initDesiredBaseTwistInBase = initDesiredState.segment<BASE_COORDINATE_SIZE>(BASE_COORDINATE_SIZE);
  const auto initDesiredBaseLinearVelocityInWorld =
      rotateVectorBaseToOrigin(getLinearVelocity(initDesiredBaseTwistInBase), getOrientation(initDesiredBasePose));

  // Compute zmp / inverted pendulum foot placement offset: delta p = sqrt(h / g) * (v - v_des)
  scalar_t pendulumFrequency = std::sqrt(settings_.invertedPendulumHeight / 9.81);
  scalar_t zmpX = pendulumFrequency * (initBaseLinearVelocityInWorld.x() - initDesiredBaseLinearVelocityInWorld.x());
  scalar_t zmpY = pendulumFrequency * (initBaseLinearVelocityInWorld.y() - initDesiredBaseLinearVelocityInWorld.y());
  const vector3_t zmpReactiveOffset = {zmpX, zmpY, 0.0};

  std::vector<ConvexTerrain> nominalFootholdTerrain;
  std::vector<vector3_t> heuristicFootholds;

  // Nominal foothold is equal to current foothold for legs in contact
  if (startsWithStancePhase(contactTimings)) {
    ConvexTerrain convexTerrain;
    convexTerrain.plane = lastContacts_[leg].second;
    nominalFootholdTerrain.push_back(convexTerrain);
    heuristicFootholds.push_back(convexTerrain.plane.positionInWorld);
  }

  // For future contact phases, use TargetTrajectories at halve the contact phase
  int contactCount = 0;
  for (const auto& contactPhase : contactTimings) {
    if (hasStartTime(contactPhase)) {
      const scalar_t timeTillContact = contactPhase.start - initTime;
      const scalar_t contactEndTime =
          hasEndTime(contactPhase) ? contactPhase.end : std::max(finalTime + settings_.referenceExtensionAfterHorizon, contactPhase.start);
      const scalar_t middleContactTime = 0.5 * (contactEndTime + contactPhase.start);

      // Get previous foothold if there was one at this time
      const FootPhase* previousIterationContact = nullptr;
      if (!feetNormalTrajectories_[leg].empty()) {
        const auto& footPhase = getFootPhase(leg, middleContactTime);
        if (footPhase.contactFlag()) {
          previousIterationContact = &footPhase;
        }
      }

      if (timeTillContact < settings_.previousFootholdTimeDeadzone && previousIterationContact != nullptr) {
        // Simply copy the information out of the previous iteration
        nominalFootholdTerrain.push_back(*previousIterationContact->nominalFootholdConstraint());
        heuristicFootholds.push_back(previousIterationContact->nominalFootholdLocation());
      } else {
        // Compute foot position from cost desired trajectory
        const vector_t state = targetTrajectories.getDesiredState(middleContactTime);
        const base_coordinate_t desiredBasePose = state.head<BASE_COORDINATE_SIZE>();
        const joint_coordinate_t desiredJointPositions = state.segment<JOINT_COORDINATE_SIZE>(2 * BASE_COORDINATE_SIZE);
        vector3_t referenceFootholdPositionInWorld =
            kinematicsModel_->footPositionInOriginFrame(leg, desiredBasePose, desiredJointPositions);

        // Add ZMP offset to the first upcoming foothold.
        if (contactCount == 0) {
          referenceFootholdPositionInWorld += zmpReactiveOffset;
        }

        // Apply Position deadzone and low pass filter w.r.t. previous foothold
        if (previousIterationContact != nullptr) {
          vector3_t previousFootholdPositionInWorld = previousIterationContact->nominalFootholdLocation();

          if ((referenceFootholdPositionInWorld - previousFootholdPositionInWorld).norm() < settings_.previousFootholdDeadzone) {
            referenceFootholdPositionInWorld = previousFootholdPositionInWorld;
          } else {
            // low pass filter
            const scalar_t lambda = settings_.previousFootholdFactor;
            referenceFootholdPositionInWorld = lambda * previousFootholdPositionInWorld + (1.0 - lambda) * referenceFootholdPositionInWorld;
          }
        }

        // kinematic penalty = w * (overExtensionTouchdown^2 + overExtensionLiftoff^2)
        const vector3_t hipInWorldTouchdown = kinematicsModel_->legRootInOriginFrame(
            leg, targetTrajectories.getDesiredState(contactPhase.start).head<BASE_COORDINATE_SIZE>());
        const vector3_t hipInWorldLiftoff =
            kinematicsModel_->legRootInOriginFrame(leg, targetTrajectories.getDesiredState(contactEndTime).head<BASE_COORDINATE_SIZE>());
        auto scoringFunction = [&](const vector3_t& footPositionInWorld) {
          return this->kinematicPenalty(footPositionInWorld, hipInWorldTouchdown, hipInWorldLiftoff);
        };

        if (contactPhase.start < finalTime) {
          ConvexTerrain convexTerrain = terrainModel.getConvexTerrainAtPositionInWorld(referenceFootholdPositionInWorld, scoringFunction);
          nominalFootholdTerrain.push_back(convexTerrain);
          heuristicFootholds.push_back(referenceFootholdPositionInWorld);
        } else {  // After the horizon -> we are only interested in the position and orientation
          ConvexTerrain convexTerrain;
          convexTerrain.plane =
              terrainModel.getLocalTerrainAtPositionInWorldAlongGravity(referenceFootholdPositionInWorld, scoringFunction);
          nominalFootholdTerrain.push_back(convexTerrain);
          heuristicFootholds.push_back(referenceFootholdPositionInWorld);
        }
      }

      ++contactCount;

      // Can stop for this leg if we have processed one contact phase after (or extending across) the horizon
      if (contactEndTime > finalTime) {
        break;
      }
    }
  }

  return {nominalFootholdTerrain, heuristicFootholds};
}

void SwingTrajectoryPlanner::adaptJointReferencesWithInverseKinematics(const inverse_kinematics_function_t& inverseKinematicsFunction,
                                                                       scalar_t finalTime) {
  for (int k = 0; k < targetTrajectories_.timeTrajectory.size(); ++k) {
    const scalar_t t = targetTrajectories_.timeTrajectory[k];

    const base_coordinate_t basePose = getBasePose(comkino_state_t(targetTrajectories_.stateTrajectory[k]));
    const vector3_t basePositionInWorld = getPositionInOrigin(basePose);
    const vector3_t eulerXYZ = getOrientation(basePose);

    for (int leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
      const auto& footPhase = this->getFootPhase(leg, t);
      const vector3_t PositionBaseToFootInWorldFrame = footPhase.getPositionInWorld(t) - basePositionInWorld;
      const vector3_t PositionBaseToFootInBaseFrame = rotateVectorOriginToBase(PositionBaseToFootInWorldFrame, eulerXYZ);

      const size_t offset = 2 * BASE_COORDINATE_SIZE + 3 * leg;
      targetTrajectories_.stateTrajectory[k].segment(offset, 3) = inverseKinematicsFunction(leg, PositionBaseToFootInBaseFrame);
    }

    // Can stop adaptation as soon as we have processed a point beyond the horizon.
    if (t > finalTime) {
      break;
    }
  }
}

void SwingTrajectoryPlanner::updateLastContact(int leg, scalar_t expectedLiftOff, const vector3_t& currentFootPosition,
                                               const TerrainModel& terrainModel) {
  // Get orientation from terrain model, position from the kinematics
  auto lastContactTerrain = terrainModel.getLocalTerrainAtPositionInWorldAlongGravity(currentFootPosition);
  lastContactTerrain.positionInWorld = currentFootPosition;
  lastContacts_[leg] = {expectedLiftOff, lastContactTerrain};
}

scalar_t SwingTrajectoryPlanner::kinematicPenalty(const vector3_t& footPositionInWorld, const vector3_t& hipInWorldAtTouchdown,
                                                  const vector3_t& hipInWorldAtLiftoff) const {
  // Touchdown
  const scalar_t legExtensionTouchdown = (footPositionInWorld - hipInWorldAtTouchdown).norm();
  const scalar_t overExtensionTouchdown = std::max(legExtensionTouchdown - settings_.nominalLegExtension, 0.0);

  // liftOff
  const scalar_t legExtensionLiftoff = (footPositionInWorld - hipInWorldAtLiftoff).norm();
  const scalar_t overExtensionLiftoff = std::max(legExtensionLiftoff - settings_.nominalLegExtension, 0.0);

  return settings_.legOverExtensionPenalty *
         (overExtensionTouchdown * overExtensionTouchdown + overExtensionLiftoff * overExtensionLiftoff);
}

std::vector<SwingPhase::SwingProfile::Node> SwingTrajectoryPlanner::extractSwingProfileFromReference(
    int leg, const SwingPhase::SwingEvent& liftoff, const SwingPhase::SwingEvent& touchdown) const {
  const scalar_t swingDuration = touchdown.time - liftoff.time;
  const vector3_t swingVector = touchdown.terrainPlane->positionInWorld - liftoff.terrainPlane->positionInWorld;
  const scalar_t swingDistance = swingVector.norm();
  vector3_t swingTrajectoryNormal = swingVector.cross(vector3_t(0.0, 0.0, 1.0).cross(swingVector)).normalized();

  // degenerate normal if there is no horizontal displacement
  if (swingVector.head<2>().norm() < 0.01) {
    swingTrajectoryNormal = vector3_t::UnitZ();
  }

  // Define where to sample the reference
  std::vector<scalar_t> samplingPhases = {0.25, 0.5, 0.75};

  std::vector<SwingPhase::SwingProfile::Node> swingNodes;
  for (const scalar_t phase : samplingPhases) {
    const scalar_t t = liftoff.time + phase * swingDuration;
    const auto& state = targetTrajectories_.getDesiredState(t);
    const auto& input = targetTrajectories_.getDesiredInput(t);

    // Compute foot position and velocity from reference desired trajectory
    const base_coordinate_t desiredBasePose = state.head<BASE_COORDINATE_SIZE>();
    const base_coordinate_t desiredBaseTwist = state.segment<BASE_COORDINATE_SIZE>(BASE_COORDINATE_SIZE);
    const joint_coordinate_t desiredJointPositions = state.segment<JOINT_COORDINATE_SIZE>(2 * BASE_COORDINATE_SIZE);
    const joint_coordinate_t desiredJointVelocities = input.segment<JOINT_COORDINATE_SIZE>(3 * NUM_CONTACT_POINTS);
    const vector3_t referenceFootpositionInWorld = kinematicsModel_->footPositionInOriginFrame(leg, desiredBasePose, desiredJointPositions);
    const vector3_t footpositionRelativeToLiftoff = referenceFootpositionInWorld - liftoff.terrainPlane->positionInWorld;
    const vector3_t referenceFootvelocityInWorld =
        kinematicsModel_->footVelocityInOriginFrame(leg, desiredBasePose, desiredBaseTwist, desiredJointPositions, desiredJointVelocities);

    // Convert to the swing profile relative coordinates
    SwingPhase::SwingProfile::Node node;
    node.phase = phase;
    if (swingDistance > 0.01) {
      node.tangentialProgress = footpositionRelativeToLiftoff.dot(swingVector) / (swingDistance * swingDistance);
      node.tangentialVelocityFactor = referenceFootvelocityInWorld.dot(swingVector) * swingDuration / (swingDistance * swingDistance);
    } else {
      node.tangentialProgress = 0.0;
      node.tangentialVelocityFactor = 0.0;
    }
    // position = liftoff + progress towards touchdown + height offset in normal direction. >> solve for height
    node.swingHeight = swingTrajectoryNormal.dot(footpositionRelativeToLiftoff - node.tangentialProgress * swingVector);
    node.normalVelocity = referenceFootvelocityInWorld.dot(swingTrajectoryNormal);

    swingNodes.push_back(node);
  }

  return swingNodes;
}

void SwingTrajectoryPlanner::setDefaultSwingProfile() {
  defaultSwingProfile_ = SwingPhase::SwingProfile();
  defaultSwingProfile_.sdfMidswingMargin = settings_.sdfMidswingMargin;
  defaultSwingProfile_.maxSwingHeightAdaptation = 2.0 * settings_.swingHeight;

  SwingPhase::SwingProfile::Node midPoint;
  midPoint.phase = 0.5;
  midPoint.swingHeight = settings_.swingHeight;
  midPoint.normalVelocity = 0.0;
  midPoint.tangentialProgress = 0.6;
  midPoint.tangentialVelocityFactor = 2.0;
  defaultSwingProfile_.nodes.push_back(midPoint);
}

SwingTrajectoryPlannerSettings loadSwingTrajectorySettings(const std::string& filename, bool verbose) {
  SwingTrajectoryPlannerSettings settings{};

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(filename, pt);

  const std::string prefix{"model_settings.swing_trajectory_settings."};

  if (verbose) {
    std::cerr << "\n #### Swing trajectory Settings:" << std::endl;
    std::cerr << " #### ==================================================" << std::endl;
  }

  ocs2::loadData::loadPtreeValue(pt, settings.liftOffVelocity, prefix + "liftOffVelocity", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.touchDownVelocity, prefix + "touchDownVelocity", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.swingHeight, prefix + "swingHeight", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.errorGain, prefix + "errorGain", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.swingTimeScale, prefix + "swingTimeScale", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.sdfMidswingMargin, prefix + "sdfMidswingMargin", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.terrainMargin, prefix + "terrainMargin", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.previousFootholdFactor, prefix + "previousFootholdFactor", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.previousFootholdDeadzone, prefix + "previousFootholdDeadzone", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.previousFootholdTimeDeadzone, prefix + "previousFootholdTimeDeadzone", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.invertedPendulumHeight, prefix + "invertedPendulumHeight", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.nominalLegExtension, prefix + "nominalLegExtension", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.legOverExtensionPenalty, prefix + "legOverExtensionPenalty", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.referenceExtensionAfterHorizon, prefix + "referenceExtensionAfterHorizon", verbose);
  ocs2::loadData::loadPtreeValue(pt, settings.swingTrajectoryFromReference, prefix + "swingTrajectoryFromReference", verbose);

  if (verbose) {
    std::cerr << " #### ==================================================" << std::endl;
  }

  return settings;
}

}  // namespace switched_model
