/**
 * @file drivers_sbus.c
 * @author zhanglongbo (zhanglongbo@lejurobot.com)
 * @brief
 * @version 1.0
 * @date 2024-04-01
 * @copyright Copyright (c) 2024
 * @note
 */
#ifndef DRIVERS_SBUS_H
#define DRIVERS_SBUS_H

#define SBUS_MODE //根据不同模块类型选择使用串口模式（UART_MODE）或SBUS模式(SBUS_MODE)
//#define DEUG_SERIAL //是否开启打印串口数据
//#define DEUG_CHANNEL //是否开启打印遥控器各通道数据

#define SERIAL_DEVICE_PATH "/dev/usb_remote" //对应实际的串口设备 

/* typedef  */
typedef unsigned           char uint8_t;
typedef unsigned short     int uint16_t;

/* SBUS 超时计时器 */
extern uint8_t g_sbus_timeout;;
/* SBUS 接收标志 */
extern uint8_t g_sbus_rec_flag;
/* SBUS 接收指示 */
extern uint8_t g_sbus_rec_index;
/* SBUS 接收缓存 */
#ifdef UART_MODE
#define BAUD_RATE B115200 //选择使用的波特率
extern uint8_t sbus_buf[35];
#endif

#ifdef SBUS_MODE
#define BAUD_RATE B100000 //选择使用的波特率
extern uint8_t sbus_buf[25];
#endif

extern int serial_port;

typedef struct
{
    uint16_t channel_1;  /*!< 通道1 */
    uint16_t channel_2;  /*!< 通道2 */
    uint16_t channel_3;  /*!< 通道3 */
    uint16_t channel_4;  /*!< 通道4 */
    uint16_t channel_5;  /*!< 通道5 */
    uint16_t channel_6;  /*!< 通道6 */
    uint16_t channel_7;  /*!< 通道7 */
    uint16_t channel_8;  /*!< 通道8 */
    uint16_t channel_9;  /*!< 通道9 */
    uint16_t channel_10; /*!< 通道10 */
    uint16_t channel_11; /*!< 通道11 */
    uint16_t channel_12; /*!< 通道12 */
    uint16_t channel_13; /*!< 通道13 */
    uint16_t channel_14; /*!< 通道14 */
    uint16_t channel_15; /*!< 通道15 */
    uint16_t channel_16; /*!< 通道16 */
    uint8_t  sbus_state; /*!< 通道状态：0未连接，1正常连接 */
} SbusInfoTypeDef;

//-----H12------
/*
channel_1	右遥感左右  282-1722
channel_2	右遥感上下  282-1722
channel_3	左遥感下上  282-1722
channel_4	左遥感左右  282-1722
channel_5	E三段开关   282/1002/1722
channel_6	F三段开关   282/1002/1722
channel_7	A二段按键   282/1722
channel_8	B二段按键   282/1722
channel_9	C二段按键   282/1722
channel_10	D二段按键   282/1722
channel_11	G滚轮       282-1722
channel_12	H滚轮       282-1722
*/

/* SBUS 信息结构体 */
extern SbusInfoTypeDef SbusRxData;

#ifdef __cplusplus
extern "C" {
#endif
int initSbus(void);
uint8_t recSbusData(void);
void parseSbusRxData(void);
void checkSbusTimeOut(void);
void printSbusInfo(SbusInfoTypeDef sbus_info);
void handle_signal(int sig);
void timer_Hander(int signum);
#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_SBUS_H */
