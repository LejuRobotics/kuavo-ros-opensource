#pragma once

#include <memory>

#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>

namespace ocs2 {
namespace humanoid {

using namespace ocs2;

class EndEffectorConstraint final : public StateConstraint{
public:
    using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
    using quaternion_t = Eigen::Quaternion<scalar_t>;

    EndEffectorConstraint(const EndEffectorKinematics<scalar_t>& endEffectorKinematics, const ReferenceManager& referenceManager, const int eeIndex);
    ~EndEffectorConstraint() override = default;
    EndEffectorConstraint* clone() const override { return new EndEffectorConstraint(*endEffectorKinematicsPtr_, *referenceManagerPtr_, eeIndex_); }

    size_t getNumConstraints(scalar_t time) const override;
    vector_t getValue(scalar_t time, const vector_t& state, const PreComputation& preComputation) const override;
    VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                             const PreComputation& preComputation) const override;
    bool isActive(scalar_t time) const override;
private:
    EndEffectorConstraint(const EndEffectorConstraint& other) = default;
    std::pair<vector_t, quaternion_t> interpolateEndEffectorPose(scalar_t time, const vector_t& state) const;

    /** Cached pointer to the pinocchio end effector kinematics. Is set to nullptr if not used. */
    PinocchioEndEffectorKinematics* pinocchioEEKinPtr_ = nullptr;

    vector3_t eeDesiredPosition_;
    quaternion_t eeDesiredOrientation_;

    std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;
    const ReferenceManager* referenceManagerPtr_;

    int eeIndex_;
    static int eeCount;
};


}  // namespace humanoid
}  // namespace ocs2

