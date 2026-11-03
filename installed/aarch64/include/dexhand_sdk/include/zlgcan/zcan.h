#pragma once

#ifdef __cplusplus
#define EXTERN_C  extern "C"
#else
#define EXTERN_C
#endif

#define ZCAN_API
#define INVALID_HANDLE_VALUE  ((HANDLE)(-1))

#define CMD_CAN_FILTER      0x14    // 滤波
#define CMD_CAN_TTX         0x16    // 定时发送
#define CMD_CAN_TTX_CTL     0x17    // 使能定时发送
#define CMD_CAN_TRES        0x18    // CAN终端电阻

#define ZCAN_CMD_SET_CHNL_RECV_MERGE    0x32    // 设置合并接收 0:不合并接收;1:合并接收
#define ZCAN_CMD_GET_CHNL_RECV_MERGE    0x33    // 获取是否开启合并接收 0:不合并接收;1:合并接收

#define CMD_SET_SN          0x42    // 获取SN号
#define CMD_GET_SN          0x43    // 设置SN号
#define CMD_CAN_TX_TIMEOUT  0x44    // 发送超时

#define ZCAN_CMD_GET_SEND_QUEUE_SIZE    0x100   // 获取队列大小，uint32_t
#define ZCAN_CMD_GET_SEND_QUEUE_SPACE   0x101   // 获取队列剩余空间, uint32_t
#define ZCAN_CMD_SET_SEND_QUEUE_CLR     0x102   // 清空发送队列,1：清空 
#define ZCAN_CMD_SET_SEND_QUEUE_EN      0x103   // 开启发送队列,1：使能 

typedef unsigned char U8;
typedef unsigned short U16;
typedef unsigned int U32;
typedef unsigned long long U64;

enum eZCANDataDEF
{
    //数据类型
    ZCAN_DT_ZCAN_CAN_DATA           = 1,    // CAN/CANFD数据
    ZCAN_DT_ZCAN_CANFD_DATA         = 2,    // CAN/CANFD数据
    ZCAN_DT_ZCAN_ERROR_DATA         = 3,    // CAN/CANFD错误数据
    ZCAN_DT_ZCAN_LIN_DATA           = 4,    // LIN数据
    ZCAN_DT_ZCAN_LIN_ERROR_DATA     = 5,    // LIN错误数据
};
#pragma pack(push, 1)

typedef enum {
    ZCAN_TX_NORM = 0, /**< normal transmission */
    ZCAN_TX_ONCE = 1, /**< single-shot transmission */
    ZCAN_SR_NORM = 2, /**< self reception */
    ZCAN_SR_ONCE = 3, /**< single-shot transmission & self reception */
} ZCAN_TX_MODE;

/** CAN filter configuration */
typedef struct {
    U8 type;    /**< 0-std_frame, 1-ext_frame */
    U8 pad[3];
    U32 sid;    /**< start-id */
    U32 eid;    /**< end-id */
} ZCAN_FILTER;

/** controller initialization */
typedef struct {
    U32 clk;    // 时钟频率
    U32 mode;   /**< bit0-normal/listen_only, bit1-ISO/BOSCH */
    struct {
        U8 tseg1;
        U8 tseg2;
        U8 sjw;
        U8 smp;
        U16 brp;
    } aset;
    struct {
        U8 tseg1;
        U8 tseg2;
        U8 sjw;
        U8 smp;
        U16 brp;
    } dset;
} ZCAN_INIT;

/** CAN message info */
typedef struct {
    U32 txm : 4; /**< TX-mode, @see ZCAN_TX_MODE */
    U32 fmt : 4; /**< 0-CAN2.0, 1-CANFD */
    U32 sdf : 1; /**< 0-data_frame, 1-remote_frame */
    U32 sef : 1; /**< 0-std_frame, 1-ext_frame */
    U32 err : 1; /**< error flag */
    U32 brs : 1; /**< bit-rate switch */
    U32 est : 1; /**< error state */
    U32 tx  : 1; /**< received valid, tx frame */
    U32 echo : 1; /**< tx valid, echo frame */
    U32 qsend_100us:1; /**< queue send delay unit, 1-100us, 0-ms */
    U32 qsend:1; /**< send valid, queue send frame */
    U32 pad : 15;
} ZCAN_MSG_INF;

/** CAN message header */
typedef struct {
    U32 ts; /**< timestamp */
    U32 id; /**< CAN-ID */
    ZCAN_MSG_INF inf; /**< @see ZCAN_MSG_INF */
    U16 pad; /**< send queue delay */
    U8 chn; /**< channel */
    U8 len; /**< data length */
} ZCAN_MSG_HDR;

/** CAN2.0-frame */
typedef struct {
    ZCAN_MSG_HDR hdr;
    U8 dat[8];
} ZCAN_20_MSG;

/** CANFD-frame */
typedef struct {
    ZCAN_MSG_HDR hdr;
    U8 dat[64];
} ZCAN_FD_MSG;

// 错误帧
typedef struct {
    ZCAN_MSG_HDR hdr;
    U8 dat[8];
} ZCAN_ERR_MSG;

// 定时发送结构体
typedef struct {
    U32 interval;       // 定时发送周期，单位 100us
    U16 repeat;         // 发送次数，0等于循环发
    U8 index;           // 定时发送列表的帧索引号，也就是第几条定时发送报文
    U8 flags;           // 0-此帧禁用定时发送，1-此帧使能定时发送
    ZCAN_FD_MSG msg;    // CANFD帧结构体
} ZCAN_TTX;

typedef struct {
    U32 size;           // 实际生效的数组的长度（用来控制实际生效多少条定时发送帧结构体）
    ZCAN_TTX table[8];  // 最大设置8条
} ZCAN_TTX_CFG;

// 设备信息
typedef struct {
    U16 hwv;    /**< hardware version */
    U16 fwv;    /**< firmware version */
    U16 drv;    /**< driver version */
    U16 api;    /**< API version */
    U16 irq;    /**< IRQ */
    U8 chn;     /**< channels */
    U8 sn[20];  /**< serial number */
    U8 id[40];  /**< card id */
    U16 pad[4]; 
} ZCAN_DEV_INF;

/** controller status */
typedef struct {
    U8 IR;      /**< not used(for backward compatibility) */
    U8 MOD;     /**< not used */
    U8 SR;      /**< not used */
    U8 ALC;     /**< not used */
    U8 ECC;     /**< not used */
    U8 EWL;     /**< not used */
    U8 RXE;     /**< RX errors */
    U8 TXE;     /**< TX errors */
    U32 PAD;
} ZCAN_STAT;

//lin begin
typedef struct _VCI_LIN_INIT_CONFIG
{
	U8    linMode;                       // 是否作为主机，0-从机，1-主机
	U8    chkSumMode;                    // 校验方式，1-经典校验 2-增强校验 3-自动(对应eZLINChkSumMode的模式)
	U16   reserved;					     // 保留
	U32   linBaud;                       // 波特率，取值1000~20000
}ZCAN_LIN_INIT_CONFIG, *PZCAN_LIN_INIT_CONFIG;

typedef struct tagZCANLINData
{
    union{
        struct {
            U8 ID : 6;          // 帧ID
            U8 Parity : 2;      // 帧ID校验
        }unionVal;
        U8 rawVal;          // 受保护的ID原始值
    } PID;                  // 受保护的ID
    struct
    {
        U64  timeStamp;         // 时间戳，单位微秒(us)
        U8    dataLen;          // 数据长度
        U8    dir;              // 传输方向，0-接收 1-发送
        U8    chkSum;           // 数据校验, 部分设备不支持校验数据的获取
        U8    reserved[13];     // 保留
        U8    data[8];          // 数据
    }RxData;                // 仅接收数据时有效
	U8 reserved[7];         // 保留
}ZCANLINData;

typedef struct tagZCANLINErrData
{
	U64  timeStamp;             // 时间戳, 单位微秒(us)
	union {
		struct {
			U8    ID : 6;       // 帧ID
			U8    Parity : 2;   // 帧ID校验
		}unionVal;
		U8    rawVal;           // 受保护的ID原始值
	}       PID;                // 受保护的ID
	U8    dataLen;
	U8    data[8];
	union
	{
		struct
		{
			U16 errStage : 4;   // 错误阶段
			U16 errReason : 4;  // 错误原因
			U16 reserved : 8;   // 保留
		};
		U16 unionErrData;
	}errData;
	U8    dir;              // 传输方向 
	U8    chkSum;           // 数据校验, 部分设备不支持校验数据的获取
	U8    reserved[10];     // 保留
}ZCANLINErrData;

typedef struct _VCI_LIN_MSG{
	U8        chnl;         // 数据通道
	U8        dataType;     // 数据类型, 0-LIN数据 1-LIN错误数据
	union
	{
		ZCANLINData             zcanLINData;        // LIN数据
		ZCANLINErrData          zcanLINErrData;     // LIN错误数据
		U8                    raw[46];              // RAW数据
	} data;                                         // 实际数据, 联合体，有效成员根据 dataType 字段而定
}ZCAN_LIN_MSG, *PZCAN_LIN_MSG;

enum eZLINChkSumMode
{
	DEFAULT = 0,        // 默认，启动时配置
	CLASSIC_CHKSUM,     // 经典校验
	ENHANCE_CHKSUM,     // 增强校验 
	AUTOMATIC,          // 自动，设备自动识别校验方式（仅ZCAN_SetLINSubscribe时有效）
};
enum
{
	MAX_LIN_ID_COUNT = 63,
	MIN_LIN_DLC = 1,
	MAX_LIN_DLC = 8,
	AUTO_LIN_DLC = 255,
};
typedef struct _VCI_LIN_PUBLISH_CFG
{
	U8    ID;               // 受保护的ID（ID取值范围为0-63）
	U8    dataLen;          // dataLen范围为1-8
	U8    data[8];
	U8    chkSumMode;       // 校验方式，0-默认，启动时配置 1-经典校验 2-增强校验(对应eZLINChkSumMode的模式)
	U8    reserved[5];      // 保留
}ZCAN_LIN_PUBLISH_CFG, *PZCAN_LIN_PUBLISH_CFG;

typedef struct _VCI_LIN_SUBSCIBE_CFG
{
	U8    ID;               // 受保护的ID（ID取值范围为0-63）
	U8    dataLen;          // dataLen范围为1-8 当为255（0xff）则表示设备自动识别报文长度
	U8    chkSumMode;       // 校验方式，0-默认，启动时配置 1-经典校验 2-增强校验 3-自动(对应eZLINChkSumMode的模式)
	U8    reserved[5];      // 保留
}ZCAN_LIN_SUBSCIBE_CFG, *PZCAN_LIN_SUBSCIBE_CFG;

// 合并接收数据数据结构，支持CAN/CANFD/LIN/错误等不同类型数据
typedef struct tagZCANDataObj
{
    U8        dataType;         // 数据类型, 参考eZCANDataDEF中 数据类型 部分定义
    U8        chnl;             // 数据通道
    union{
        struct{
            U16 reserved : 16;                      // 保留
        }unionVal;
        U16 rawVal;
    }flag;                                          // 标志信息, 暂未使用
    U8        extraData[4];                         // 额外数据, 暂未使用
    union
    {
        ZCAN_20_MSG             zcanCANData;        // CAN数据
        ZCAN_FD_MSG             zcanCANFDData;      // CANFD数据
        ZCAN_ERR_MSG            zcanErrData;        // 错误数据
        ZCANLINData             zcanLINData;        // LIN数据
        ZCANLINErrData          zcanLINErrData;     // LIN错误数据
        U8                      raw[92];            // RAW数据
    } data;                                         // 实际数据, 联合体，有效成员根据 dataType 字段而定
}ZCANDataObj;


// UDS传输协议版本
typedef U8 ZCAN_UDS_TRANS_VER;
#define ZCAN_UDS_TRANS_VER_0        0       // ISO15765-2(2004版本)
#define ZCAN_UDS_TRANS_VER_1        1       // ISO15765-2(2016版本)

// 帧类型
typedef U8 ZCAN_UDS_FRAME_TYPE;
#define ZCAN_UDS_FRAME_CAN          0       // CAN帧
#define ZCAN_UDS_FRAME_CANFD        1       // CANFD帧
#define ZCAN_UDS_FRAME_CANFD_BRS    2       // CANFD加速帧

// UDS填充模式
#define FILL_MODE_SHORT    0    // 小于8字节填充至8字节，大于8字节时按DLC就近填充
#define FILL_MODE_NONE     1    // 不填充
#define FILL_MODE_MAX      2    // 填充至最大数据长度 (不建议)

// CAN UDS请求数据
typedef struct _ZCAN_UDS_REQUEST
{
    U32 req_id;                             // 请求事务ID，范围0~65535，本次请求的唯一标识
    U8 channel;                             // 设备通道索引 0~255
    ZCAN_UDS_FRAME_TYPE frame_type;         // 帧类型
    U8 reserved0[2];                        // 保留
    U32 src_addr;                           // 请求地址
    U32 dst_addr;                           // 响应地址
    U8 suppress_response;                   // 1:抑制响应
    U8 sid;                                 // 请求服务id
    U8 reserved1[6];                        // 保留
    struct {
        U32 timeout;                        // 响应超时时间(ms)。因PC定时器误差，建议设置不小于200ms
        U32 enhanced_timeout;               // 收到消极响应错误码为0x78后的超时时间(ms)。因PC定时器误差，建议设置不小于200ms
        U8 check_any_negative_response:1;   // 接收到非本次请求服务的消极响应时是否需要判定为响应错误
        U8 wait_if_suppress_response:1;     // 抑制响应时是否需要等待消极响应，等待时长为响应超时时间
        U8 flag:6;                          // 保留
        U8 reserved0[7];                    // 保留
    } session_param;                        // 会话层参数
    struct {
        ZCAN_UDS_TRANS_VER version;       // 传输协议版本, VERSION_0, VERSION_1
        U8 max_data_len;                  // 单帧最大数据长度, can:8, canfd:64
        U8 local_st_min;                  // 本程序发送流控时用，连续帧之间的最小间隔, 0x00-0x7F(0ms~127ms), 0xF1-0xF9(100us~900us)
        U8 block_size;                    // 流控帧的块大小
        U8 fill_byte;                     // 无效字节的填充数据
        U8 ext_frame;                     // 0:标准帧 1:扩展帧
        U8 is_modify_ecu_st_min;          // 是否忽略ECU返回流控的STmin，强制使用本程序设置的 remote_st_min
        U8 remote_st_min;                 // 发送多帧时用, is_ignore_ecu_st_min = 1 时有效, 0x00-0x7F(0ms~127ms), 0xF1-0xF9(100us~900us)
        U32 fc_timeout;                   // 接收流控超时时间(ms), 如发送首帧后需要等待回应流控帧
        U8 fill_mode;                     // 数据长度填充模式, FILL_MODE_SHORT, FILL_MODE_NONE, FILL_MODE_MAX
        U8 reserved0[3];                  // 保留
    } trans_param;                        // 传输层参数
    U8 *data;           // 数据数组(不包含SID)
    U32 data_len;       // 数据数组的长度
    U32 reserved2;      // 保留
} ZCAN_UDS_REQUEST;

// UDS错误码
typedef U8 ZCAN_UDS_ERROR;
#define ZCAN_UDS_ERROR_OK                   0    // 没错误
#define ZCAN_UDS_ERROR_TIMEOUT              1    // 响应超时
#define ZCAN_UDS_ERROR_TRANSPORT            2    // 发送数据失败
#define ZCAN_UDS_ERROR_CANCEL               3    // 取消请求
#define ZCAN_UDS_ERROR_SUPPRESS_RESPONSE    4    // 抑制响应
#define ZCAN_UDS_ERROR_BUSY                 5    // 忙碌中
#define ZCAN_UDS_ERROR_REQ_PARAM            6    // 请求参数错误
#define ZCAN_UDS_ERROR_OTHTER               100

typedef U8 ZCAN_UDS_RESPONSE_TYPE;
#define ZCAN_UDS_RT_NEGATIVE 0              // 消极响应
#define ZCAN_UDS_RT_POSITIVE 1              // 积极响应

// UDS响应数据
typedef struct _ZCAN_UDS_RESPONSE
{
    ZCAN_UDS_ERROR status;              // 响应状态
    U8 reserved[6];                     // 保留
    ZCAN_UDS_RESPONSE_TYPE type;        // 响应类型
    union {
        struct {
            U8 sid;             // 响应服务id
            U32 data_len;       // 数据长度(不包含SID), 数据存放在接口传入的dataBuf中
        } positive;
        struct {
            U8  neg_code;       // 固定为0x7F
            U8  sid;            // 请求服务id
            U8  error_code;     // 错误码
        } negative;
        U8 raw[8]; 
    };
} ZCAN_UDS_RESPONSE;

// UDS控制类型
typedef U32 ZCAN_UDS_CTRL_CODE;
#define ZCAN_UDS_CTRL_STOP_REQ 0            // 停止UDS请求

// UDS控制请求
typedef struct _ZCAN_UDS_CTRL_REQ
{
    U32 reqID;                      // 请求事务ID，指明要操作哪一条请求
	ZCAN_UDS_CTRL_CODE cmd;         // 控制类型
    U8 reserved[8];                 // 保留
} ZCAN_UDS_CTRL_REQ;

// UDS控制结果
typedef U32 ZCAN_UDS_CTRL_RESULT;
#define ZCAN_UDS_CTRL_RESULT_OK  0      // 成功
#define ZCAN_UDS_CTRL_RESULT_ERR 1      // 失败

// UDS控制响应数据
typedef struct _ZCAN_UDS_CTRL_RESP
{
    ZCAN_UDS_CTRL_RESULT result;    // 操作结果
    U8 reserved[8];                 // 保留
} ZCAN_UDS_CTRL_RESP;

#pragma pack(pop)

// 
EXTERN_C U32 ZCAN_API VCI_OpenDevice(U32 Type, U32 Card, U32 Reserved);
EXTERN_C U32 ZCAN_API VCI_CloseDevice(U32 Type, U32 Card);
EXTERN_C U32 ZCAN_API VCI_InitCAN(U32 Type, U32 Card, U32 Port, ZCAN_INIT *pInit);
EXTERN_C U32 ZCAN_API VCI_ReadBoardInfo(U32 Type, U32 Card, ZCAN_DEV_INF *pInfo);
EXTERN_C U32 ZCAN_API VCI_ReadErrInfo(U32 Type, U32 Card, U32 Port, ZCAN_ERR_MSG *pErr);
EXTERN_C U32 ZCAN_API VCI_ReadCANStatus(U32 Type, U32 Card, U32 Port, ZCAN_STAT *pStat);
EXTERN_C U32 ZCAN_API VCI_GetReference(U32 Type, U32 Card, U32 Port, U32 Ref, void *pData);
EXTERN_C U32 ZCAN_API VCI_SetReference(U32 Type, U32 Card, U32 Port, U32 Ref, void *pData);
EXTERN_C U32 ZCAN_API VCI_GetReceiveNum(U32 Type, U32 Card, U32 Port);
EXTERN_C U32 ZCAN_API VCI_ClearBuffer(U32 Type, U32 Card, U32 Port);
EXTERN_C U32 ZCAN_API VCI_StartCAN(U32 Type, U32 Card, U32 Port);
EXTERN_C U32 ZCAN_API VCI_ResetCAN(U32 Type, U32 Card, U32 Port);
EXTERN_C U32 ZCAN_API VCI_Transmit(U32 Type, U32 Card, U32 Port, ZCAN_20_MSG *pData, U32 Count);
EXTERN_C U32 ZCAN_API VCI_TransmitFD(U32 Type, U32 Card, U32 Port, ZCAN_FD_MSG *pData, U32 Count);
EXTERN_C U32 ZCAN_API VCI_Receive(U32 Type, U32 Card, U32 Port, ZCAN_20_MSG *pData, U32 Count, U32 Time);
EXTERN_C U32 ZCAN_API VCI_ReceiveFD(U32 Type, U32 Card, U32 Port, ZCAN_FD_MSG *pData, U32 Count, U32 Time);
EXTERN_C U32 ZCAN_API VCI_Debug(U32 Debug);

// LIN
EXTERN_C U32 VCI_InitLIN(U32 Type, U32 Card, U32 LinChn, PZCAN_LIN_INIT_CONFIG pLINInitConfig);
EXTERN_C U32 VCI_StartLIN(U32 Type, U32 Card, U32 LinChn);
EXTERN_C U32 VCI_ResetLIN(U32 Type, U32 Card, U32 LinChn);
EXTERN_C U32 VCI_TransmitLIN(U32 Type, U32 Card, U32 LinChn, PZCAN_LIN_MSG pSend, U32 Len);
EXTERN_C U32 VCI_GetLINReceiveNum(U32 Type, U32 Card, U32 LinChn);
EXTERN_C U32 VCI_ClearLINBuffer(U32 Type, U32 Card, U32 LinChn);
EXTERN_C U32 VCI_ReceiveLIN(U32 Type, U32 Card, U32 LinChn, PZCAN_LIN_MSG pReceive, U32 Len,int WaitTime);
EXTERN_C U32 VCI_SetLINSubscribe(U32 Type, U32 Card, U32 LinChn, PZCAN_LIN_SUBSCIBE_CFG pSend, U32 nSubscribeCount);
EXTERN_C U32 VCI_SetLINPublish(U32 Type, U32 Card, U32 LinChn, PZCAN_LIN_PUBLISH_CFG pSend, U32 nPublishCount);

// 合并接收
EXTERN_C U32 VCI_TransmitData(unsigned Type, unsigned Card, unsigned Port, ZCANDataObj *pData, unsigned Count);
EXTERN_C U32 VCI_ReceiveData(unsigned Type, unsigned Card, unsigned Port, ZCANDataObj *pData, unsigned Count, unsigned Time);

// UDS
EXTERN_C U32 VCI_UDS_Request(unsigned Type, unsigned Card, const ZCAN_UDS_REQUEST *req, ZCAN_UDS_RESPONSE *resp, U8 *dataBuf, U32 dataBufSize);
EXTERN_C U32 VCI_UDS_Control(unsigned Type, unsigned Card, const ZCAN_UDS_CTRL_REQ *ctrl, ZCAN_UDS_CTRL_RESP *resp);
