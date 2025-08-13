#ifndef OBJECTDICTIONARY_H
#define OBJECTDICTIONARY_H

/*-------------SHARE-------------------------*/
#define INDEX_POSITION_ACTUAL_VALUE 0x6064
#define INDEX_MOTOR_RATED_CURRENT 0x6075 //额定电流


/*-------------YOUDA-------------------------*/
#define YD_VENDOR_ID 0x0000005A
#define YD_PRODUCT_CODE 0x00000003

#define INDEX_PRODUCT_CODE 0x1026 //厂商识别码，未确定
#define TORQUE_MODE_VELOCITY_LIMIT 0x3401  //力矩模式速度限制
#define DRIVER_RATING_COUNT 0x6075  //关节额定电流值
#define FORWARD_ROTATION_DIRECTION 0x3104 //电机正转方向

#define DRIVER_WORK_MODE 0x3507  //驱动器工作模式
#define JOINT_CSP_KP 0X3500   //位置环P增益
#define JOINT_CSV_KP 0x3504   //速度环P增益
#define JOINT_CSV_KI 0x3505   //速度环积分增益
#define JOINT_CSP_OFFSET 0x3502  //位置环前馈
#define JOINT_CSP_COMMAND_FILTER 0x3532  //位置指令滤波
#define ENCODER_FEEDBACK_MODE 0x381C  //编码器工作模式
#define MOTOR_PARAMETER_CODE 0x3E01 //电机参数代码
#define DRIVER_SAVE_SETTING_PARAMETER 0x313D  //驱动器保存参数


/*-------------ELMO-------------------------*/
#define ELMO_VENDOR_ID 0x0000009A
#define ELMO_PRODUCT_CODE 0x00030924

/*------ elmo error index-------*/
#define SENSOR_FEEDBACK_ERROR 0x2311 // 编码器反馈错误
#define FEEDBACK_ERROR 0x7300        // 反馈错误
#define SPEED_TRACKING_ERROR 0x8480  // 速度跟踪超限
#define SPEED_LIMIT_EXCEEDED 0x8481
#define POSITION_TRACKING_ERROR 0x8611 // 位置跟踪超限
#define FAILED_TO_START_MOTOR 0XFF10   // 电机启动错误

#endif