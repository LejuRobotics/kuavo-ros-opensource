#ifndef _actuators_interface_h_
#define _actuators_interface_h_

#include <stdio.h>
#include <stdint.h>
#include <math.h>
// #include "elmo_motor.h"
#include "EcDemoPlatform.h"
#include "EcDemoApp.h"

using JointParam_t = MotorParam_t;

typedef struct
{
  int (*init)(uint32_t num_actuators, std::string& ecmaster_driver_type);
  void (*deInit)(void);
  void (*setJointOffset)(double_t *offset, uint16_t len);
  void (*setJointPosition)(const uint16_t *ids,const EcMasterType* driver, uint32_t num, MotorParam_t *params);
  void (*setJointVelocity)(const uint16_t *ids,const EcMasterType* driver, uint32_t num, MotorParam_t *params);
  void (*setJointTorque)(const uint16_t *ids,const EcMasterType* driver, uint32_t num, MotorParam_t *params);
  void (*setEncoderRange)(uint32_t *encoderRange, uint16_t len);

  void (*getJointData)(const uint16_t *ids,const EcMasterType* driver, uint32_t num, MotorParam_t *data);
  void (*addIgnore)(const uint16_t *ids, uint32_t num);

  void (*setJointKp)(const std::vector<int32_t>& joint_kp);
  void (*setJointKd)(const std::vector<int32_t>& joint_kd);
  
  /**
   * @brief 读取电机Kp/Kd
   * 
   * @param ids 
   * @param driver_type 
   * @param joint_kp 
   * @return int 0 表示成功，1 表示驱动类型不支持该操作，2 表示读取失败
   */
  int (*readJointKp)(const std::vector<uint16_t> &ids, EcMasterType driver_type, std::vector<int32_t>& joint_kp);
  int (*readJointKd)(const std::vector<uint16_t> &ids, EcMasterType driver_type, std::vector<int32_t>& joint_kd);

  /**
   * @brief 写入电机Kp/Kd
   * 
   * @param ids 
   * @param driver_type 
   * @param joint_kp 
   * @return int 0 表示成功，1 表示驱动类型不支持该操作，2 表示写入失败
   */
  int (*writeJointKp)(const std::vector<uint16_t> &ids, EcMasterType driver_type, const std::vector<int32_t>& joint_kp);
  int (*writeJointKd)(const std::vector<uint16_t> &ids, EcMasterType driver_type, const std::vector<int32_t>& joint_kd);

} ActuatorsInterface_t;

int ECMaster_init(uint32_t num_actuators, std::string& driver_type);
int8_t actuatorsInterfaceSetup(const char *type, ActuatorsInterface_t *interfacePtr);

#endif
