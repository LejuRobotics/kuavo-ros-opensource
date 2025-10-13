#ifndef __USER_APP_WORKPD_H
#define __USER_APP_WORKPD_H

#include <stdint.h>
#include "EcMotor.h"
#include "EcType.h"

struct _T_EC_DEMO_APP_CONTEXT;
typedef struct _T_EC_DEMO_APP_CONTEXT T_EC_DEMO_APP_CONTEXT;

void addMotorInterpTask(uint16_t id, EcMasterType* driver, double targetPos, double totalTime, double dt);
void addMotorSinTask(const uint16_t* ids, uint32_t num, const MotorCspSinParam_t& param);
void addMotorPositionControlTask(const uint16_t* ids, uint32_t num, const MotorPositionControlParam_t& param);
void addMotorActionSequenceTask(const uint16_t* ids, uint32_t num, const MotorActionSequenceParam_t& param);
void addMotorDualLegActionSequenceTask(const uint16_t* left_ids, const uint16_t* right_ids, uint32_t num, const MotorDualLegActionSequenceParam_t& param);
void addMotorActionFrameTask(const uint16_t* ids, uint32_t num, const MotorActionFrameParam_t& param);
void addMotorMultiActionFrameTask(const MotorMultiActionFrameParam_t* param);

EC_T_DWORD UserAppWorkpd(T_EC_DEMO_APP_CONTEXT* pAppContext);

#endif
