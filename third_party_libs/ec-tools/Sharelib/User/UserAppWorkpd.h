#ifndef __USER_APP_WORKPD_H
#define __USER_APP_WORKPD_H

#include <stdint.h>
#include "EcMotor.h"
#include "EcType.h"

struct _T_EC_DEMO_APP_CONTEXT;
typedef struct _T_EC_DEMO_APP_CONTEXT T_EC_DEMO_APP_CONTEXT;

void addMotorInterpTask(uint16_t id, double targetPos, double totalTime, double dt);
void addMotorSinTask(const uint16_t* ids, uint32_t num, const MotorCspSinParam_t& param);
void addMotorMultiActionTask(const uint16_t* ids, uint32_t num, const MotorMultiActionParam_t& param);

EC_T_DWORD UserAppWorkpd(T_EC_DEMO_APP_CONTEXT* pAppContext);

#endif
