
#pragma once

namespace ocs2
{
namespace humanoid
{

class FloatInterpolation
{
public:
    FloatInterpolation(PinocchioInterface pinocchio_interface, CentroidalModelInfo modelInfo)
    : pinocchio_interface_(pinocchio_interface), modelInfo_(modelInfo)
    {
        ikSolver_.setParam(std::make_shared<PinocchioInterface>(pinocchio_interface_), std::make_shared<CentroidalModelInfo>(modelInfo_));

        for(int i=0; i<footFrameNames_.size(); i++) {
            footFrameIds_.push_back(pinocchio_interface_.getModel().getFrameId(footFrameNames_[i]));
        }
    }

    // 设置浮动基的位姿和单腿关节角度，返回左右腿的末端执行器位姿
    std::pair<vector3_t, matrix3_t> getSingleFootPose(const vector_t& init_q, int legIndex);
    std::pair<vector3_t, matrix3_t> getBasePose(const vector_t& init_q);


    vector_t getlegJointAngles(const vector_t& observationState, const vector6_t& basePose, 
                               const vector_t& desired_leg_q);

private:
    InverseKinematics ikSolver_;
    PinocchioInterface pinocchio_interface_;
    const CentroidalModelInfo modelInfo_;
    std::vector<std::string> footFrameNames_ = {"leg_l6_link", "leg_r6_link"}; // 左右腿末端执行器的frame名称
    std::vector<size_t> footFrameIds_; // 左右腿末端执行器的frame id
    std::string baseFrameNames_ = {"base_link"}; // 浮动基的frame名称
};


}  // namespace humanoid
}  // namespace ocs2