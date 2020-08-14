//
// Created by rgrandia on 17.02.20.
//

#include "ocs2_quadruped_interface/QuadrupedInterface.h"

#include <ocs2_core/misc/Display.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_switched_model_interface/core/SwitchedModelStateEstimator.h>
#include <ocs2_switched_model_interface/foot_planner/SwingTrajectoryPlanner.h>
#include <ocs2_switched_model_interface/logic/ModeSequenceTemplate.h>
#include <ocs2_switched_model_interface/terrain/PlanarTerrainModel.h>

namespace switched_model {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
QuadrupedInterface::QuadrupedInterface(const kinematic_model_t& kinematicModel, const ad_kinematic_model_t& adKinematicModel,
                                       const com_model_t& comModel, const ad_com_model_t& adComModel, const std::string& pathToConfigFolder)

    : kinematicModelPtr_(kinematicModel.clone()), comModelPtr_(comModel.clone()) {
  loadSettings(pathToConfigFolder + "/task.info");
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto QuadrupedInterface::loadCostMatrices(const std::string& pathToConfigFile, const kinematic_model_t& kinematicModel,
                                          const state_vector_t& initialState)
    -> std::tuple<state_matrix_t, input_matrix_t, state_matrix_t> {
  state_matrix_t Q;
  input_matrix_t R;
  state_matrix_t QFinal;

  // cost function components
  ocs2::loadData::loadEigenMatrix(pathToConfigFile, "Q", Q);
  ocs2::loadData::loadEigenMatrix(pathToConfigFile, "R", R);
  ocs2::loadData::loadEigenMatrix(pathToConfigFile, "Q_final", QFinal);

  // costs over Cartesian velocities
  Eigen::Matrix<scalar_t, 12, 12> J_allFeet;
  for (int leg = 0; leg < 4; ++leg) {
    Eigen::Matrix<double, 6, 12> J_thisfoot = kinematicModel.baseToFootJacobianInBaseFrame(leg, getJointPositions(initialState));
    J_allFeet.block<3, 12>(3 * leg, 0) = J_thisfoot.bottomRows<3>();
  }
  R.block<12, 12>(12, 12) = (J_allFeet.transpose() * R.block<12, 12>(12, 12) * J_allFeet).eval();
  return {Q, R, QFinal};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void QuadrupedInterface::loadSettings(const std::string& pathToConfigFile) {
  rolloutSettings_.loadSettings(pathToConfigFile, "slq.rollout");
  modelSettings_ = loadModelSettings(pathToConfigFile);

  // partitioning times
  size_t numPartitions;
  ocs2::loadData::loadPartitioningTimes(pathToConfigFile, timeHorizon_, numPartitions, partitioningTimes_, true);

  // initial state of the switched system
  Eigen::Matrix<scalar_t, RBD_STATE_DIM, 1> initRbdState;
  ocs2::loadData::loadEigenMatrix(pathToConfigFile, "initialRobotState", initRbdState);
  SwitchedModelStateEstimator switchedModelStateEstimator(*comModelPtr_);
  initialState_ = switchedModelStateEstimator.estimateComkinoModelState(initRbdState);

  // Gait Schedule
  const auto initModeSchedule = loadModeSchedule(pathToConfigFile, "initialModeSchedule", false);
  const auto defaultModeSequenceTemplate = loadModeSequenceTemplate(pathToConfigFile, "defaultModeSequenceTemplate", false);
  const auto defaultGait = [&] {
    Gait gait{};
    gait.duration = defaultModeSequenceTemplate.switchingTimes.back();
    // Events: from time -> phase
    std::for_each(defaultModeSequenceTemplate.switchingTimes.begin() + 1, defaultModeSequenceTemplate.switchingTimes.end() - 1,
                  [&](double eventTime) { gait.eventPhases.push_back(eventTime / gait.duration); });
    // Modes:
    gait.modeSequence = defaultModeSequenceTemplate.modeSequence;
    return gait;
  }();

  GaitSchedule gaitSchedule{0.0, defaultGait};

  // Swing trajectory planner
  const auto swingTrajectorySettings = loadSwingTrajectorySettings(pathToConfigFile);
  SwingTrajectoryPlanner swingTrajectoryPlanner{swingTrajectorySettings, getComModel(), getKinematicModel()};

  // Terrain
  auto loadedTerrain = loadTerrainPlane(pathToConfigFile, true);
  std::unique_ptr<TerrainModel> terrainModel(new PlanarTerrainModel(std::move(loadedTerrain)));

  // Mode schedule manager
  modeScheduleManagerPtr_ = std::make_shared<SwitchedModelModeScheduleManager>(std::move(gaitSchedule), std::move(swingTrajectoryPlanner),
                                                                               std::move(terrainModel));

  // Display
  std::cerr << "\nTime Partition: {" << ocs2::toDelimitedString(partitioningTimes_) << "}\n";
  std::cerr << "\nInitial Modes Schedule: \n" << initModeSchedule << std::endl;
  std::cerr << "\nDefault Modes Sequence Template: \n" << defaultModeSequenceTemplate << std::endl;
}

}  // namespace switched_model
