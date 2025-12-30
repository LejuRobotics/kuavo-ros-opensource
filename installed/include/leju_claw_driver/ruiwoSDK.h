#ifndef RUIWO_SDK_H
#define RUIWO_SDK_H
#ifdef __cplusplus
extern "C" 
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include "bmapi.h"

#define TEST_MSG_RX_TIMEOUT 1000

// 宏定义控制打印
#define PRINT_SEND_DATA 0
#define PRINT_MOTOR_DATA 0
#define PRINT_RECEIVE_DATA 0

typedef struct {
    char dev_type[10];
    int dev_channel;
    int bit_rate;
    int data_rate;
    int terminal_res;
    int timeout;
} DevInfo;

typedef struct {
    float CAN_COM_THETA_MIN;
    float CAN_COM_THETA_MAX;
    float CAN_COM_VELOCITY_MIN;
    float CAN_COM_VELOCITY_MAX;
    float CAN_COM_TORQUE_MIN;
    float CAN_COM_TORQUE_MAX;
    float CAN_COM_POS_KP_MIN;
    float CAN_COM_POS_KP_MAX;
    float CAN_COM_POS_KD_MIN;
    float CAN_COM_POS_KD_MAX;
    float CAN_COM_VEL_KP_MIN;
    float CAN_COM_VEL_KP_MAX;
    float CAN_COM_VEL_KD_MIN;
    float CAN_COM_VEL_KD_MAX;
    float CAN_COM_VEL_KI_MIN;
    float CAN_COM_VEL_KI_MAX;
} CanComParam;

typedef struct {
    BM_ChannelHandle channel;
    BM_NotificationHandle notification;
    DevInfo dev_info;
    CanComParam can_com_param;
} RUIWOTools;

int open_canbus(RUIWOTools* ruiwo, int init_bmlib, unsigned char *version);
int close_canbus(RUIWOTools* ruiwo);
uint32_t float_to_int(float float_num, float min_num, float max_num, uint8_t bit_num);
float int_to_float(uint32_t int_num, float min_num, float max_num, uint8_t bit_num);
void return_motor_state(BM_CanMessageTypeDef* feedback_frame, float* state_list, CanComParam* params);
int enter_motor_state(RUIWOTools* ruiwo, uint32_t dev_id, float* state_list);
int enter_reset_state(RUIWOTools* ruiwo, uint32_t dev_id, float* state_list);
int set_zero_position(RUIWOTools* ruiwo, uint32_t dev_id, float* state_list);
int run_servo_mode(RUIWOTools* ruiwo, uint32_t dev_id, float pos, float vel, float pos_kp, float pos_kd, float vel_kp, float vel_kd, float vel_ki, float* state_list);
int run_ptm_mode(RUIWOTools* ruiwo, uint32_t dev_id, float pos, float vel, float pos_kp, float pos_kd, float torque, float* state_list);
int run_ptm_mode_No_response(RUIWOTools* ruiwo, uint32_t dev_id, float pos, float vel, float pos_kp, float pos_kd, float torque, float* state_list);
int run_vel_mode(RUIWOTools* ruiwo, uint32_t dev_id, float vel, float vel_kp, float vel_kd, float vel_ki, float* state_list);
int run_torque_mode(RUIWOTools* ruiwo, uint32_t dev_id, float torque, float* state_list);
void initialize_ruiwoSDK(RUIWOTools *ruiwo, bool is_claw);
#ifdef __cplusplus
}
#endif
#endif // RUIWO_SDK_H

