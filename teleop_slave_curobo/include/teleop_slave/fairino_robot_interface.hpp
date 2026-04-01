#pragma once

#include <memory>
#include <string>

#include "robot.h"

class IFairinoRobot {
public:
    virtual ~IFairinoRobot() = default;

    virtual errno_t RPC(const char* ip) = 0;
    virtual errno_t CloseRPC() = 0;
    virtual errno_t SetReConnectParam(bool enable, int reconnect_time, int period) = 0;
    virtual errno_t ResetAllError() = 0;
    virtual errno_t Mode(int mode) = 0;
    virtual errno_t RobotEnable(uint8_t state) = 0;
    virtual errno_t SetSpeed(int vel) = 0;
    virtual errno_t ServoMoveStart() = 0;
    virtual errno_t ServoMoveEnd() = 0;
    virtual errno_t MoveJ(JointPos* joint_pos, int tool, int user, float vel, float acc,
                          float ovl, ExaxisPos* epos, float blend_t, uint8_t offset_flag,
                          DescPose* offset_pos) = 0;
    virtual errno_t ServoJ(JointPos* joint_pos, ExaxisPos* axis_pos, float acc, float vel,
                           float cmd_t, float filter_t, float gain, int id) = 0;
    virtual errno_t ServoCart(int mode, DescPose* desc_pose, ExaxisPos axis_pos, float pos_gain[6],
                              float acc, float vel, float cmd_t, float filter_t, float gain) = 0;
    virtual errno_t GetActualJointPosDegree(uint8_t flag, JointPos* jpos) = 0;
    virtual errno_t GetActualTCPPose(uint8_t flag, DescPose* desc_pos) = 0;
    virtual errno_t GetActualTCPNum(uint8_t flag, int* id) = 0;
    virtual errno_t GetActualWObjNum(uint8_t flag, int* id) = 0;
    virtual errno_t GetInverseKinHasSolution(int type, DescPose* desc_pos, JointPos* joint_pos_ref,
                                             uint8_t* result) = 0;
    virtual errno_t GetInverseKinRef(int type, DescPose* desc_pos, JointPos* joint_pos_ref,
                                     JointPos* joint_pos) = 0;
    virtual errno_t GetRobotErrorCode(int* maincode, int* subcode) = 0;
};

class FairinoRobotSdkAdapter : public IFairinoRobot {
public:
    FairinoRobotSdkAdapter() = default;
    ~FairinoRobotSdkAdapter() override = default;

    errno_t RPC(const char* ip) override;
    errno_t CloseRPC() override;
    errno_t SetReConnectParam(bool enable, int reconnect_time, int period) override;
    errno_t ResetAllError() override;
    errno_t Mode(int mode) override;
    errno_t RobotEnable(uint8_t state) override;
    errno_t SetSpeed(int vel) override;
    errno_t ServoMoveStart() override;
    errno_t ServoMoveEnd() override;
    errno_t MoveJ(JointPos* joint_pos, int tool, int user, float vel, float acc,
                  float ovl, ExaxisPos* epos, float blend_t, uint8_t offset_flag,
                  DescPose* offset_pos) override;
    errno_t ServoJ(JointPos* joint_pos, ExaxisPos* axis_pos, float acc, float vel,
                   float cmd_t, float filter_t, float gain, int id) override;
    errno_t ServoCart(int mode, DescPose* desc_pose, ExaxisPos axis_pos, float pos_gain[6],
                      float acc, float vel, float cmd_t, float filter_t, float gain) override;
    errno_t GetActualJointPosDegree(uint8_t flag, JointPos* jpos) override;
    errno_t GetActualTCPPose(uint8_t flag, DescPose* desc_pos) override;
    errno_t GetActualTCPNum(uint8_t flag, int* id) override;
    errno_t GetActualWObjNum(uint8_t flag, int* id) override;
    errno_t GetInverseKinHasSolution(int type, DescPose* desc_pos, JointPos* joint_pos_ref,
                                     uint8_t* result) override;
    errno_t GetInverseKinRef(int type, DescPose* desc_pos, JointPos* joint_pos_ref,
                             JointPos* joint_pos) override;
    errno_t GetRobotErrorCode(int* maincode, int* subcode) override;

private:
    FRRobot robot_;
};

std::unique_ptr<IFairinoRobot> CreateFairinoRobotSdkAdapter();
