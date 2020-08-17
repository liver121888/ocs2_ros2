#pragma once

#include <ocs2_switched_model_interface/constraint/EndEffectorConstraint.h>

namespace switched_model {

using EndEffectorPositionConstraintSettings = EndEffectorConstraintSettings;

class EndEffectorPositionConstraint : public EndEffectorConstraint {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = EndEffectorConstraint;
  using typename BASE::ad_com_model_t;
  using typename BASE::ad_interface_t;
  using typename BASE::ad_kinematic_model_t;
  using typename BASE::ad_scalar_t;
  using typename BASE::ad_vector_t;
  using typename BASE::constraint_timeStateInput_matrix_t;
  using typename BASE::input_matrix_t;
  using typename BASE::input_state_matrix_t;
  using typename BASE::input_vector_t;
  using typename BASE::LinearApproximation_t;
  using typename BASE::QuadraticApproximation_t;
  using typename BASE::state_matrix_t;
  using typename BASE::state_vector_t;
  using typename BASE::timeStateInput_matrix_t;
  using settings_t = EndEffectorPositionConstraintSettings;

  explicit EndEffectorPositionConstraint(int legNumber, settings_t settings, ad_com_model_t& adComModel,
                                         ad_kinematic_model_t& adKinematicsModel, bool generateModels = true,
                                         std::string constraintPrefix = "o_EEPositionConstraint_")
      : BASE(ocs2::ConstraintOrder::Linear, std::move(constraintPrefix), legNumber, std::move(settings), adComModel, adKinematicsModel,
             EndEffectorPositionConstraint::adfunc, generateModels) {}

  EndEffectorPositionConstraint(const EndEffectorPositionConstraint& rhs) = default;

  EndEffectorPositionConstraint* clone() const override { return new EndEffectorPositionConstraint(*this); }

 private:
  static void adfunc(ad_com_model_t& adComModel, ad_kinematic_model_t& adKinematicsModel, int legNumber, const ad_vector_t& tapedInput,
                     ad_vector_t& o_footPosition) {
    // Extract elements from taped input
    ad_scalar_t t = tapedInput(0);
    comkino_state_ad_t x = tapedInput.segment(1, STATE_DIM);

    // Extract elements from state
    const base_coordinate_ad_t comPose = getComPose(x);
    const joint_coordinate_ad_t qJoints = getJointPositions(x);

    // Get base state from com state
    const base_coordinate_ad_t basePose = adComModel.calculateBasePose(comPose);
    o_footPosition = adKinematicsModel.footPositionInOriginFrame(legNumber, basePose, qJoints);
  };
};
}  // namespace switched_model
