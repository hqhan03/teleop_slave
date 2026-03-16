#include "teleop_slave/fairino_robot_interface.hpp"

errno_t FairinoRobotSdkAdapter::RPC(const char* ip) {
    return robot_.RPC(ip);
}

errno_t FairinoRobotSdkAdapter::CloseRPC() {
    return robot_.CloseRPC();
}

errno_t FairinoRobotSdkAdapter::SetReConnectParam(bool enable, int reconnect_time, int period) {
    return robot_.SetReConnectParam(enable, reconnect_time, period);
}

errno_t FairinoRobotSdkAdapter::ResetAllError() {
    return robot_.ResetAllError();
}

errno_t FairinoRobotSdkAdapter::Mode(int mode) {
    return robot_.Mode(mode);
}

errno_t FairinoRobotSdkAdapter::RobotEnable(uint8_t state) {
    return robot_.RobotEnable(state);
}

errno_t FairinoRobotSdkAdapter::SetSpeed(int vel) {
    return robot_.SetSpeed(vel);
}

errno_t FairinoRobotSdkAdapter::ServoMoveStart() {
    return robot_.ServoMoveStart();
}

errno_t FairinoRobotSdkAdapter::ServoMoveEnd() {
    return robot_.ServoMoveEnd();
}

errno_t FairinoRobotSdkAdapter::ServoJ(JointPos* joint_pos, ExaxisPos* axis_pos, float acc, float vel,
                                       float cmd_t, float filter_t, float gain, int id) {
    return robot_.ServoJ(joint_pos, axis_pos, acc, vel, cmd_t, filter_t, gain, id);
}

errno_t FairinoRobotSdkAdapter::GetActualJointPosDegree(uint8_t flag, JointPos* jpos) {
    return robot_.GetActualJointPosDegree(flag, jpos);
}

errno_t FairinoRobotSdkAdapter::GetActualTCPPose(uint8_t flag, DescPose* desc_pos) {
    return robot_.GetActualTCPPose(flag, desc_pos);
}

errno_t FairinoRobotSdkAdapter::GetActualTCPNum(uint8_t flag, int* id) {
    return robot_.GetActualTCPNum(flag, id);
}

errno_t FairinoRobotSdkAdapter::GetActualWObjNum(uint8_t flag, int* id) {
    return robot_.GetActualWObjNum(flag, id);
}

errno_t FairinoRobotSdkAdapter::GetInverseKinHasSolution(int type, DescPose* desc_pos,
                                                         JointPos* joint_pos_ref, uint8_t* result) {
    return robot_.GetInverseKinHasSolution(type, desc_pos, joint_pos_ref, result);
}

errno_t FairinoRobotSdkAdapter::GetInverseKinRef(int type, DescPose* desc_pos,
                                                 JointPos* joint_pos_ref, JointPos* joint_pos) {
    return robot_.GetInverseKinRef(type, desc_pos, joint_pos_ref, joint_pos);
}

errno_t FairinoRobotSdkAdapter::GetRobotErrorCode(int* maincode, int* subcode) {
    return robot_.GetRobotErrorCode(maincode, subcode);
}

std::unique_ptr<IFairinoRobot> CreateFairinoRobotSdkAdapter() {
    return std::make_unique<FairinoRobotSdkAdapter>();
}
