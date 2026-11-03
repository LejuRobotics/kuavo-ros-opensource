/*-----------------------------------------------------------------------------
 * EcLink.h
 * Copyright                acontis technologies GmbH, Ravensburg, Germany
 * Response                 Stefan Zintgraf
 * Description              EtherCAT Master link layer interface
 *---------------------------------------------------------------------------*/

#ifndef INC_ECLINK
#define INC_ECLINK

/*-INCLUDES------------------------------------------------------------------*/
#if (!defined INC_ECOS) && (!defined INC_LINK_OS_LAYER)
#error EcOs.h / LinkOsLayer.h include missing!
#endif
#ifndef INC_ECINTERFACECOMMON
#include "EcInterfaceCommon.h"
#endif
#ifndef INC_ECTYPE
#include "EcType.h"
#endif
#ifndef INC_ECVERSION
#include "EcVersion.h"
#endif
#ifndef INC_ECERROR
#include "EcError.h"
#endif
#ifndef INC_ECLOG
#include "EcLog.h"
#endif

#ifndef ATEMLL_API
#ifdef __cplusplus
#define ATEMLL_API extern "C"
#else
#define ATEMLL_API
#endif
#endif

/*-DEFINES-------------------------------------------------------------------*/
#define LINK_LAYER_DRV_DESC_PATTERN         (EC_T_DWORD)0x11aaddaa
#define LINK_LAYER_DRV_DESC_VERSION         (EC_T_DWORD)0x00031000      /* version 3.1 */

#define EC_LINK_PARMS_SIGNATURE_PATTERN     (EC_T_DWORD)0xBA100000      /* Mask 0xfff00000 */
#define EC_LINK_PARMS_SIGNATURE_VERSION     (EC_T_DWORD)0x00010000      /* Version 1, mask 0x000f0000 */
#define EC_LINK_PARMS_SIGNATURE (EC_T_DWORD)(EC_LINK_PARMS_SIGNATURE_PATTERN|EC_LINK_PARMS_SIGNATURE_VERSION)

#define MAX_DRIVER_IDENT_LEN                40              /* deprecated */
#define EC_DRIVER_IDENT_MAXLEN              39              /* maximum driver ident string length */
#define EC_DRIVER_IDENT_NAMESIZE            (EC_DRIVER_IDENT_MAXLEN + 1) /* driver ident string is zero terminated */

#define EC_LINKUNIT_PCILOCATION             0x01000000      /* if the MSB of dwUnit is set to this value, the 3LSB mean Bus-Dev-Func (if supported by LinkOsLayer) */

#define EC_LINKNOTIFY_GENERIC               0x00000000
#define EC_LINKNOTIFY_LINKSTATECHGD         (EC_LINKNOTIFY_GENERIC | 0x1)   /* not used within EC-Master */
#define EC_LINKNOTIFY_TXMEMORYCHGD          (EC_LINKNOTIFY_GENERIC | 0x2)   /* not used within EC-Master */

#define EC_LINKIOCTL_GENERIC                0x00000000
#define EC_LINKIOCTL_USER                   0x00F00000                      /* for user extension */
#define EC_LINKIOCTL_INTERRUPTENABLE        (EC_LINKIOCTL_GENERIC  | 0x1)   /* not used within EC-Master */
#define EC_LINKIOCTL_PROMISCOUSMODE         (EC_LINKIOCTL_GENERIC  | 0x2)   /* not used within EC-Master */
#define EC_LINKIOCTL_SETMULTICASTADDR       (EC_LINKIOCTL_GENERIC  | 0x3)   /* not used within EC-Master */
#define EC_LINKIOCTL_CLRMULTICASTADDR       (EC_LINKIOCTL_GENERIC  | 0x4)   /* not used within EC-Master */
#define EC_LINKIOCTL_SETLINKMODE            (EC_LINKIOCTL_GENERIC  | 0x5)   /* not used within EC-Master */
#define EC_LINKIOCTL_GETLINKMODE            (EC_LINKIOCTL_GENERIC  | 0x6)   /* not used within EC-Master */
#define EC_LINKIOCTL_RESTART                (EC_LINKIOCTL_GENERIC  | 0x7)   /* not used within EC-Master */
#define EC_LINKIOCTL_SET_LINKMODE           (EC_LINKIOCTL_GENERIC  | 0x8)   /* not used within EC-Master: set mode to polling or interrupt */
#define EC_LINKIOCTL_SET_ALLOC_SENDFRAME    (EC_LINKIOCTL_GENERIC  | 0x9)   /* not used within EC-Master: for debug purposes: enable/disable EcLinkAllocSendFrame() support */
#define EC_LINKIOCTL_FORCELINKMODE          (EC_LINKIOCTL_GENERIC  | 0xA)   /* not used within EC-Master: Bit 0=Autoneg.;1=FullDuplex;16-31=LinkSpeed 1=10,2=100,3=1000MBit */
#define EC_LINKIOCTL_GETINFOLIST            (EC_LINKIOCTL_GENERIC  | 0xB)   /* not used within EC-Master: query EC_T_LINK_INFOLIST */

#define EC_LINKENABLED_OFF 0
#define EC_LINKENABLED_ON 1
#define EC_LINKENABLED_ONLY_SEND 2
#define EC_LINKENABLED_ONLY_RECEIVE 3
#define EC_LINKIOCTL_SET_LINKENABLED        (EC_LINKIOCTL_GENERIC  | 0xC)   /* enables or disables the link */

/* The following IO-Controls are mandatory for LinkLayer in interrupt mode */
#define EC_LINKIOCTL_REGISTER_FRAME_CLBK    (EC_LINKIOCTL_GENERIC  | 0x10)  /* registers the receive frame callback */
#define EC_LINKIOCTL_UNREGISTER_FRAME_CLBK  (EC_LINKIOCTL_GENERIC  | 0x11)  /* unregisters the receive frame callback */
#define EC_LINKIOCTL_INTSTATUS_READ         (EC_LINKIOCTL_GENERIC  | 0x12)  /* not used within EC-Master: read card's interrupt status register */
#define EC_LINKIOCTL_INTSTATUS_WRITE        (EC_LINKIOCTL_GENERIC  | 0x13)  /* not used within EC-Master: write card's interrupt status register */

#define EC_LINKIOCTL_FEATURECONTROL         (EC_LINKIOCTL_GENERIC  | 0x14)  /* not used within EC-Master: Bit 0=JumboFrame, Bit 1=ResetOndisconnect */

#define EC_LINKIOCTL_UPDATE_LINKSTATUS      (EC_LINKIOCTL_GENERIC  | 0x15)  /* Update link status (CPSW, eTSEC, GEM, SHEth only) */
#define EC_LINKIOCTL_GET_ETHERNET_ADDRESS   (EC_LINKIOCTL_GENERIC  | 0x23)  /* calls EcLinkGetEthernetAddress */
#define EC_LINKIOCTL_IS_REPEAT_CNT_SUPPORTED (EC_LINKIOCTL_GENERIC | 0x24)  /* return EC_E_NOERROR if supported, EC_E_NOTSUPPORTED otherwise */

#define EC_LINKIOCTL_PHY_READ               (EC_LINKIOCTL_GENERIC | 0x25)  /* not used within EC-Master */
#define EC_LINKIOCTL_PHY_WRITE              (EC_LINKIOCTL_GENERIC | 0x26)  /* not used within EC-Master */

/* LinkLayer timer support */
#define  EC_LINKIOCTL_SETTIME               (EC_LINKIOCTL_GENERIC  | 0x30)
#define  EC_LINKIOCTL_GETTIME               (EC_LINKIOCTL_GENERIC  | 0x31)
#define  EC_LINKIOCTL_WAITFORTIME           (EC_LINKIOCTL_GENERIC  | 0x32)

/* Frame type differentiation */
#define  EC_LINKIOCTL_PROCESSCYCLICFRAMES   (EC_LINKIOCTL_GENERIC  | 0x33)
#define  EC_LINKIOCTL_PROCESSACYCLICFRAMES  (EC_LINKIOCTL_GENERIC  | 0x34)
#define  EC_LINKIOCTL_SENDCYCLICFRAMES      (EC_LINKIOCTL_GENERIC  | 0x35) /* prepares link layer for sending cyclic frames */
#define  EC_LINKIOCTL_SENDACYCLICFRAMES     (EC_LINKIOCTL_GENERIC  | 0x36) /* prepares link layer for sending acyclic frames */
#define  EC_LINKIOCTL_IS_FRAMETYPE_REQUIRED (EC_LINKIOCTL_GENERIC  | 0x37) /* EC_TRUE: Master stack calls EC_LINKIOCTL_SENDCYCLICFRAMES or EC_LINKIOCTL_SENDACYCLICFRAMES once before LinkSendFrame() */

/* Frame flushing (frames are queue and sent after flushing) */
#define  EC_LINKIOCTL_FLUSHFRAMES             (EC_LINKIOCTL_GENERIC | 0x38) /* flush cyclic / acyclic frames queued at link layer (see also EC_LINKIOCTL_SENDCYCLICFRAMES, EC_LINKIOCTL_SENDACYCLICFRAMES) */
#define  EC_LINKIOCTL_IS_FLUSHFRAMES_REQUIRED (EC_LINKIOCTL_GENERIC | 0x39) /* EC_TRUE: Master stack calls EC_LINKIOCTL_FLUSHFRAMES after all cyclic frames were sent and after all acyclic frames were sent */

#define  EC_LINKIOCTL_SET_LINKSTATUS         (EC_LINKIOCTL_GENERIC | 0x3A) /* changes link status if this IO is supported. EC_T_LINKSTATUS used as parameter*/
#define  EC_LINKIOCTL_SET_FORCEDISCONNECTION (EC_LINKIOCTL_GENERIC | 0x3B)

#define  EC_LINKIOCTL_GET_SPEED              (EC_LINKIOCTL_GENERIC | 0x3C)
#define  EC_LINKIOCTL_IS_UDP                 (EC_LINKIOCTL_GENERIC | 0x3D)
#define  EC_LINKIOCTL_SET_MSECCOUNTERPTR     (EC_LINKIOCTL_GENERIC | 0x3E)
#define  EC_LINKIOCTL_GET_ICSS_STASTISTICS   (EC_LINKIOCTL_GENERIC | 0x3F)
#define  EC_LINKIOCTL_GET_NDIS_VERSION       (EC_LINKIOCTL_GENERIC | 0x40)

#define  EC_LINKIOCTL_GET_LOGLEVEL           (EC_LINKIOCTL_GENERIC | 0x41)
#define  EC_LINKIOCTL_SET_LOGLEVEL           (EC_LINKIOCTL_GENERIC | 0x42)

#define EC_LINKIOCTL_GET_ONTIMER_CYCLETIME   (EC_LINKIOCTL_GENERIC | 0x43)
#define EC_LINKIOCTL_SET_ONTIMER_CYCLETIME   (EC_LINKIOCTL_GENERIC | 0x44)
#define EC_LINKIOCTL_ONTIMER                 (EC_LINKIOCTL_GENERIC | 0x45)

/*-TYPEDEFS------------------------------------------------------------------*/
typedef enum _EC_T_LINKMODE
{
    EcLinkMode_UNDEFINED = 0,
    EcLinkMode_INTERRUPT = 1,
    EcLinkMode_POLLING   = 2,

    /* Borland C++ datatype alignment correction */
    EcLinkMode_BCppDummy = 0xFFFFFFFF
} EC_T_LINKMODE;

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_LINKLAYER_MODE_DESC
{
    EC_T_LINKMODE    eLinkMode;                  /**< [out] Operation mode of main interface */
    EC_T_LINKMODE    eLinkModeRed;               /**< [out] Operation mode of redundancy interface */
} EC_PACKED_API EC_T_LINKLAYER_MODE_DESC;
#include EC_PACKED_INCLUDESTOP

typedef enum _EC_T_LINKSTATUS
{
    eLinkStatus_UNDEFINED    = 0,
    eLinkStatus_OK           = 1,
    eLinkStatus_DISCONNECTED = 2,
    eLinkStatus_HALFDUPLEX   = 3,

    /* Borland C++ datatype alignment correction */
    eLinkStatus_BCppDummy    = 0xFFFFFFFF
} EC_T_LINKSTATUS;

typedef enum _EC_T_ALLOCSENDFRAME_MODE
{
    EcLinkAllocSendFrameMode_UNDEFINED = 0,
    EcLinkAllocSendFrameMode_ENABLED   = 1,
    EcLinkAllocSendFrameMode_DISABLED  = 2,

    /* Borland C++ datatype alignment correction */
    EcLinkAllocSendFrameMode_BCppDummy = 0xFFFFFFFF
} EC_T_ALLOCSENDFRAMEMODE;

/* PHY Interface */
typedef enum _EC_T_PHYINTERFACE
{
    ePHY_UNDEFINED  = 0,      /**< undefined */
    ePHY_FIXED_LINK = 1 << 0, /**< No PHY access at all */
    ePHY_MII        = 1 << 1, /**< MII 10 / 100 MBit */
    ePHY_RMII       = 1 << 2, /**< Reduced MII, 10 / 100 MBit */
    ePHY_GMII       = 1 << 3, /**< Gigabit MII, 10, 100, 1000 MBit */
    ePHY_SGMII      = 1 << 4, /**< Serial (SERDES) Gigabit MII, 10, 100, 1000 MBit */
    ePHY_RGMII      = 1 << 5, /**< Reduced Gigabit MII, 10, 100, 1000 MBit */
    ePHY_OSDRIVER   = 1 << 6, /**< Get interface type from OS */
    ePHY_RMII_50MHZ = 1 << 7, /**< ePHY_RMII with 50 MHz clock mode */

    /* Borland C++ datatype alignment correction */
    ePHY_BCppDummy  = 0xFFFFFFFF
} EC_T_PHYINTERFACE;

struct _EC_T_LINK_NOTIFYPARMS;
struct _EC_T_LINK_FRAMEDESC;

typedef EC_T_DWORD (*EC_T_LINK_GETTIMESTAMP)(EC_T_PVOID pCallerData, EC_T_DWORD* pdwHostTimeLo);
typedef EC_T_DWORD (*EC_T_RECEIVEFRAMECALLBACK)(EC_T_VOID* pvContext, struct _EC_T_LINK_FRAMEDESC* pLinkFrameDesc, EC_T_BOOL* pbFrameProcessed);
typedef EC_T_DWORD (*EC_T_LINK_NOTIFY)(EC_T_DWORD dwCode, struct _EC_T_LINK_NOTIFYPARMS* pParms);

/* EtherCAT notify parameters */
#include EC_PACKED_INCLUDESTART(4)
typedef struct _EC_T_LINK_NOTIFYPARMS
{
    EC_T_VOID*      pvContext;          /**< [in]  context */
    EC_T_BYTE*      pbyInBuf;           /**< [in]  input data buffer */
    EC_T_DWORD      dwInBufSize;        /**< [in]  size of input data buffer in byte */
    EC_T_BYTE*      pbyOutBuf;          /**< [out] output data buffer */
    EC_T_DWORD      dwOutBufSize;       /**< [in]  size of output data buffer in byte */
    EC_T_DWORD*     pdwNumOutData;      /**< [out] number of output data bytes stored in output data buffer */
} EC_PACKED_API EC_T_LINK_NOTIFYPARMS;
#include EC_PACKED_INCLUDESTOP

/** \defgroup ECAT_LINK_FRAMEFLAGS Link frame descriptor flags
@{
*/
#define ECAT_LINK_FRAMEFLAG_ADD_INTERFRAME_GAP      ((EC_T_DWORD)0x00000001)
#define ECAT_LINK_FRAMEFLAG_CLOSE_JUNCTION_RED_PORT ((EC_T_DWORD)0x00000002)
#define ECAT_LINK_FRAMEFLAG_OPEN_JUNCTION_RED_PORT  ((EC_T_DWORD)0x00000004)
/**@}*/

#include EC_PACKED_API_INCLUDESTART
/*****************************************************************************/
/** \brief  Frame Descriptor
*/
typedef struct _EC_T_LINK_FRAMEDESC
{
    EC_T_VOID*          pvContext;          /**< Link Layer context */

    EC_T_BYTE*          pbyFrame;           /**< Frame data buffer */
    EC_T_DWORD          dwSize;             /**< Frame data buffer size */

    EC_T_BOOL           bBuffersFollow;     /**< If EC_TRUE try to queue next frame in link layer,
                                                if EC_FALSE fill up DMA descriptors to force immediate send */

    EC_T_DWORD*         pdwTimeStampLo;     /**< Timestamp buffer */
    EC_T_DWORD*         pdwTimeStampPostLo; /**< Timestamp buffer */
    EC_T_PVOID          pvTimeStampCtxt;    /**< Context for pfnTimeStamp */
    EC_T_LINK_GETTIMESTAMP pfnTimeStamp;    /**< Get timestamp function (optional) */
    EC_T_DWORD*         pdwLastTSResult;    /**< Get timestamp result buffer */

    EC_T_WORD           wTimestampOffset;   /**< Timestamp location in frame data buffer as byte offset */
    EC_T_WORD           wTimestampSize;     /**< Timestamp size in bytes */
    EC_T_UINT64         qwTimestamp;        /**< Send or receive timestamp in ns */

    EC_T_DWORD          dwRepeatCnt;        /**< Repeat count, if 0 or 1 send once */
    EC_T_DWORD          dwFlags;            /**< Link frame flags, see ECAT_LINK_FRAME_FLAG_... */
} EC_PACKED_API EC_T_LINK_FRAMEDESC;
#include EC_PACKED_INCLUDESTOP

/* Structure to register the Link Layer receive callback */
#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_LINK_FRM_RECV_CLBK
{
    EC_T_RECEIVEFRAMECALLBACK   pfFrameReceiveCallback;
    EC_T_VOID*                  pvDevice;
} EC_PACKED_API EC_T_LINK_FRM_RECV_CLBK;
#include EC_PACKED_INCLUDESTOP

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_LINK_IOCTLPARMS
{
    EC_T_PBYTE      pbyInBuf;           /**< [in]  input data buffer */
    EC_T_DWORD      dwInBufSize;        /**< [in]  size of input data buffer in byte */
    EC_T_PBYTE      pbyOutBuf;          /**< [out] output data buffer */
    EC_T_DWORD      dwOutBufSize;       /**< [in]  size of output data buffer in byte */
    EC_T_DWORD*     pdwNumOutData;      /**< [out] number of output data bytes stored in output data buffer */
} EC_PACKED_API EC_T_LINK_IOCTLPARMS;
#include EC_PACKED_INCLUDESTOP

/* PHY write desc */
#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_LINK_PHY_WRITE_DESC
{
    EC_T_BYTE       byReg;              /**< [in] PHY register */
    EC_T_WORD       wVal;               /**< [in] value */
} EC_PACKED_API EC_T_LINK_PHY_WRITE_DESC;
#include EC_PACKED_INCLUDESTOP

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_LINK_PARMS
{
    EC_T_DWORD      dwSignature;                             /**< [in] Signature of the adapter specific structure containing the EC_T_LINK_PARMS structure */
    EC_T_DWORD      dwSize;                                  /**< [in] Size of the adapter specific structure containing the EC_T_LINK_PARMS structure */
    EC_T_LOG_PARMS  LogParms;                                /**< [in] Logging parameters */
    EC_T_CHAR       szDriverIdent[EC_DRIVER_IDENT_NAMESIZE]; /**< [in] Name of Link Layer module (driver identification) for Link Layer Selection */
    EC_T_DWORD      dwInstance;                              /**< [in] Instance of the adapter. if EC_LINKUNIT_PCILOCATION is set: contains PCI address */
    EC_T_LINKMODE   eLinkMode;                               /**< [in] Mode of operation */
    EC_T_CPUSET     cpuIstCpuAffinityMask;                   /**< [in] Interrupt service thread CPU affinity mask */
    EC_T_DWORD      dwIstPriority;                           /**< [in] Task priority of the interrupt service task (not used in polling mode) */
} EC_PACKED_API EC_T_LINK_PARMS;
#include EC_PACKED_INCLUDESTOP

/**
* \typedef PF_EcLinkOpen
* \brief Open Link Layer connection.
* \param pvLinkParms                [in] Pointer to link parameters
* \param pfReceiveFrameCallback     [in] Pointer to Rx callback function
* \param pfLinkNotifyCallback       [in] Pointer to notification callback function
* \param pContext                   [in] Context pointer. This pointer is used as parameter when the callback function is called
* \param ppvInstance                [out] Instance handle
* \return EC_E_NOERROR or error code
*/
typedef EC_T_DWORD (*PF_EcLinkOpen)(EC_T_VOID* pvLinkParms, EC_T_RECEIVEFRAMECALLBACK pfReceiveFrameCallback, EC_T_LINK_NOTIFY pfLinkNotifyCallback, EC_T_VOID* pvContext, EC_T_VOID** ppvInstance);
/**
* \typedef PF_EcLinkClose
* \brief Close Link Layer connection
* \param pvInstance                [in] Instance handle
* \return EC_E_NOERROR or error code
*/
typedef EC_T_DWORD (*PF_EcLinkClose)(EC_T_VOID* pvInstance);
/**
* \typedef PF_EcLinkSendFrame
* \brief Send data frame
* \param pvInstance                [in] Instance handle
* \param pLinkFrameDesc            [in] Pointer to the link frame descriptor
* \return EC_E_NOERROR or error code
*/
typedef EC_T_DWORD (*PF_EcLinkSendFrame)(EC_T_VOID* pvInstance, EC_T_LINK_FRAMEDESC* pLinkFrameDesc);
/**
* \typedef PF_EcLinkSendAndFreeFrame
* \brief Send data frame and free the frame buffer. This function must be supported if EcLinkAllocSendFrame() is supported
* \param pvInstance                [in] Instance handle
* \param pLinkFrameDesc            [in] Pointer to the link frame descriptor
* \return EC_E_NOERROR or error code
*/
typedef EC_T_DWORD (*PF_EcLinkSendAndFreeFrame)(EC_T_VOID* pvInstance, EC_T_LINK_FRAMEDESC* pLinkFrameDesc);
/**
* \typedef PF_EcLinkRecvFrame
* \brief Poll for received frame
* \param pvInstance                [in] Instance handle
* \param pLinkFrameDesc            [in] Pointer to the link frame descriptor
* \return EC_E_NOERROR or error code
*/
typedef EC_T_DWORD (*PF_EcLinkRecvFrame)(EC_T_VOID* pvInstance, EC_T_LINK_FRAMEDESC* pLinkFrameDesc);
/**
* \typedef PF_EcLinkAllocSendFrame
* \brief Allocate a frame buffer used for send.  If the link layer doesn't support frame allocation, this function must return EC_E_NOTSUPPORTED
* \param pvInstance                [in] Instance handle
* \param pLinkFrameDesc            [in/out] Pointer to the link frame descriptor
* \param dwSize                    [in] Size of the frame to allocate
* \return EC_E_NOERROR or error code
*/
typedef EC_T_DWORD (*PF_EcLinkAllocSendFrame)(EC_T_VOID* pvInstance, EC_T_LINK_FRAMEDESC* pLinkFrameDesc, EC_T_DWORD dwSize);
/**
* \typedef PF_EcLinkFreeSendFrame
* \brief Release a frame buffer previously allocated with EcLinkAllocSendFrame()
* \param pvInstance                [in] Instance handle
* \param pLinkFrameDesc            [in] Pointer to the link frame descriptor
* \return EC_E_NOERROR or error code
*/
typedef EC_T_VOID  (*PF_EcLinkFreeSendFrame )(EC_T_VOID* pvInstance, EC_T_LINK_FRAMEDESC* pLinkFrameDesc);
/**
* \typedef PF_EcLinkFreeRecvFrame
* \brief Release a frame buffer given to the EtherCAT master through the receive callback function
* \param pvInstance                [in] Instance handle
* \param pLinkFrameDesc            [in] Pointer to the link frame descriptor
* \return EC_E_NOERROR or error code
*/
typedef EC_T_VOID  (*PF_EcLinkFreeRecvFrame )(EC_T_VOID* pvInstance, EC_T_LINK_FRAMEDESC* pLinkFrameDesc);
/**
* \typedef PF_EcLinkGetEthernetAddress
* \brief Get Link Layer MAC address
* \param pvInstance                  [in] Instance handle
* \param pbyEthernetAddress          [out] Ethernet MAC address
* \return EC_E_NOERROR or error code
*/
typedef EC_T_DWORD      (*PF_EcLinkGetEthernetAddress)(EC_T_VOID* pvInstance, EC_T_BYTE* pbyEthernetAddress);
/**
* \typedef PF_EcLinkGetStatus
* \brief Get current link status
* \param pvInstance                  [in] Instance handle
* \return Current link status.
*/
typedef EC_T_LINKSTATUS (*PF_EcLinkGetStatus)(EC_T_VOID* pvInstance);
/**
* \typedef PF_EcLinkGetSpeed
* \brief Get current link speed
* \param pvInstance                  [in] Instance handle
* \return Current link speed.
*/
typedef EC_T_DWORD      (*PF_EcLinkGetSpeed)(EC_T_VOID* pvInstance);
/**
* \typedef PF_EcLinkGetMode
* \brief Get current link mode
* \param pvInstance                  [in] Instance handle
* \return Current link mode.
*/
typedef EC_T_LINKMODE   (*PF_EcLinkGetMode)(EC_T_VOID* pvInstance);
/**
* \typedef PF_EcLinkIoctl
* \brief Multi Purpose LinkLayer IOCTL
* \param pvInstance                  [in] Instance handle
* \param dwCode                      [in] Control code
* \param pParms                      [in/out] Pointer to the IOCTL parameters
* \return EC_E_NOERROR or error code
*/
typedef EC_T_DWORD      (*PF_EcLinkIoctl)(EC_T_VOID* pvInstance, EC_T_DWORD dwCode, EC_T_LINK_IOCTLPARMS* pParms );

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_LINK_DRV_DESC
{
    EC_T_DWORD                  dwValidationPattern;           /**< Link Layer driver validation pattern */
    EC_T_DWORD                  dwInterfaceVersion;            /**< Link Layer driver interface version */
    EC_T_LOG_PARMS              LogParms;                      /**< Log parameters structure */

    PF_EcLinkOpen               pfEcLinkOpen;                  /**< Function pointer to open link */
    PF_EcLinkClose              pfEcLinkClose;                 /**< Function pointer to close link */

    /* Standard send functions */
    PF_EcLinkSendFrame          pfEcLinkSendFrame;             /**< Function pointer to send frame */
    PF_EcLinkSendAndFreeFrame   pfEcLinkSendAndFreeFrame;      /**< Function pointer to send and free frame */

    /* Timestamping time optimized send functions */
    PF_EcLinkSendFrame          pfEcLinkRes1;                  
    PF_EcLinkSendAndFreeFrame   pfEcLinkRes2;                  

    PF_EcLinkRecvFrame          pfEcLinkRecvFrame;             /**< Function pointer to receive frame */
    PF_EcLinkAllocSendFrame     pfEcLinkAllocSendFrame;        /**< Function pointer to allocate a frame buffer used to send  */
    PF_EcLinkFreeSendFrame      pfEcLinkFreeSendFrame ;        /**< Function pointer to release a frame buffer previously allocated with AllocSendframe() */
    PF_EcLinkFreeRecvFrame      pfEcLinkFreeRecvFrame ;        /**< Function pointer to release a frame buffer given to the EtherCAT master through the receive callback function */
    PF_EcLinkGetEthernetAddress pfEcLinkGetEthernetAddress;    /**< Function pointer to get Link Layer MAC address */
    PF_EcLinkGetStatus          pfEcLinkGetStatus;             /**< Function pointer to get current link status */
    PF_EcLinkGetSpeed           pfEcLinkGetSpeed;              /**< Function pointer to get current link speed */
    PF_EcLinkGetMode            pfEcLinkGetMode;               /**< Function pointer to get current link mode */
    PF_EcLinkIoctl              pfEcLinkIoctl;                 /**< Function pointer to a multi-purpose Link Layer IOCTL */

    EC_T_VOID*                  pvLinkInstance;                /**< Pointer to the Link Layer Object/Instance */
} EC_PACKED_API EC_T_LINK_DRV_DESC;
#include EC_PACKED_INCLUDESTOP

typedef EC_T_DWORD (*PF_DOINT_HDL)(EC_T_PVOID pvLinkParms);

#define EC_LINK_INFO_DESCRIPTION_SIZE_MAX   63
typedef enum _EC_T_LINK_INFO_DATATYPE
{
    eLinkInfoDataType_bool  = 0,
    eLinkInfoDataType_byte  = 1,
    eLinkInfoDataType_word  = 2,
    eLinkInfoDataType_dword = 3,
    eLinkInfoDataType_qword = 4,

    /* Borland C++ datatype alignment correction */
    eLinkInfoDataType_BCppDummy = 0xFFFFFFFF
} EC_T_LINK_INFO_DATATYPE;

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_LINK_INFO
{
    EC_T_CHAR               szDescription[EC_LINK_INFO_DESCRIPTION_SIZE_MAX];  /*< Description text of the value */
    EC_T_LINK_INFO_DATATYPE eDataType;          /*< Datatype */
    union _EC_T_LINK_INFO_DATA
    {
        EC_T_BOOL           bData;              /*< Data as boolean */
        EC_T_BYTE           byData;             /*< Data as byte */
        EC_T_WORD           wData;              /*< Data as word */
        EC_T_DWORD          dwData;             /*< Data as dword */
        EC_T_UINT64         qwData;             /*< Data as qword */
    } EC_PACKED_API u;
} EC_PACKED_API EC_T_LINK_INFO;
#include EC_PACKED_INCLUDESTOP

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_LINK_INFOLIST
{
    EC_T_DWORD          dwInfoCntMax;       /*< Total number of info elements available */
    EC_T_DWORD          dwInfoCntUsed;      /*< Number of info elements used */
    EC_T_LINK_INFO      aInfo[1];
} EC_PACKED_API EC_T_LINK_INFOLIST;
#include EC_PACKED_INCLUDESTOP

/*****************************************************************************/
/** \brief  Variables to identify the EOE link layer driver/instance which shall
*           be opened with the EoELinkOpen() call.
*
*   \note Parameters not used for identification of the link layer (or master stack  instance) must be cleared to 0.
*/
#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_LINK_OPENPARMS_EOE
{
    EC_T_DWORD dwEmInstanceID;      /**< Instance ID, identical to the instance ID of the EtherCAT master intended to open. */
    EC_T_PVOID pvUplinkInstance;    /**< Pointer to the ``CEcEoEUplink`` instance/object (if available).*/
    EC_T_BYTE abyEthAddress[6];     /**< Ethernet address of the driver/adapter */
    EC_T_BYTE abyIpAdress[4];       /**< IP address of the driver/adapter */
    EC_T_CHAR szEoEDrvIdent[EC_DRIVER_IDENT_NAMESIZE];  /**< Name of the driver/adapter */
} EC_PACKED_API EC_T_LINK_OPENPARMS_EOE;
#include EC_PACKED_INCLUDESTOP

typedef EC_T_VOID(*EC_T_LINK_TTS_CALLBACK)(EC_T_VOID*);

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_LINK_TTS
{
    EC_T_BOOL                   bEnabled;               /**< [in] Use Time-Triggered Send */
    EC_T_DWORD                  dwCycleTimeUsec;        /**< [in] Cycle time between 2 pfnStartCycle calls */
    EC_T_DWORD                  dwSendOffsetUsec;       /**< [in] Time between pfnStartCycle call and frame transmission */
    EC_T_LINK_TTS_CALLBACK      pfnStartCycle;          /**< [in] Callback function called cyclically according dwCycleTimeUsec */
    EC_T_VOID*                  pvStartCycleContext;    /**< [in] Context passed to each pfnTtsStartCycle call */
} EC_PACKED_API EC_T_LINK_TTS;
#include EC_PACKED_INCLUDESTOP


/*****************************************************************************/
/* Hardware specific link layer parameters                                   */
/*****************************************************************************/

/* WinPcap Windows */
/* =============== */
#define EC_LINK_PARMS_SIGNATURE_WINPCAP_PATTERN (EC_T_DWORD)0x0000CA00
#define EC_LINK_PARMS_SIGNATURE_WINPCAP_VERSION (EC_T_DWORD)0x00000002
#define EC_LINK_PARMS_SIGNATURE_WINPCAP (EC_T_DWORD)(EC_LINK_PARMS_SIGNATURE|EC_LINK_PARMS_SIGNATURE_WINPCAP_PATTERN|EC_LINK_PARMS_SIGNATURE_WINPCAP_VERSION)
#define EC_LINK_PARMS_IDENT_WINPCAP "Pcap"

#define MAX_LEN_WINPCAP_ADAPTER_ID  ((EC_T_DWORD)39)
#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_LINK_PARMS_WINPCAP
{
    EC_T_LINK_PARMS linkParms;                          /**< Common link parameters. Signature must be set to EC_LINK_PARMS_SIGNATURE_WINPCAP */

    EC_T_BYTE abyIpAddress[4];    /**< IP address */
    EC_T_CHAR szAdapterId[MAX_LEN_WINPCAP_ADAPTER_ID]; /**< Adapter ID, format: {XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX} */
    EC_T_BOOL bFilterInput;                            /**< Filter input if EC_TRUE. This is needed on some system if the winpcap library notify the sent frames to the network adapter */
} EC_PACKED_API EC_T_LINK_PARMS_WINPCAP;
#include EC_PACKED_INCLUDESTOP

/* SNARF VxWorks */
/* ============= */
#define EC_LINK_PARMS_SIGNATURE_SNARF_PATTERN (EC_T_DWORD)0x0000CA10
#define EC_LINK_PARMS_SIGNATURE_SNARF_VERSION (EC_T_DWORD)0x00000002
#define EC_LINK_PARMS_SIGNATURE_SNARF (EC_T_DWORD)(EC_LINK_PARMS_SIGNATURE|EC_LINK_PARMS_SIGNATURE_SNARF_PATTERN|EC_LINK_PARMS_SIGNATURE_SNARF_VERSION)
#define EC_LINK_PARMS_IDENT_SNARF "Snarf"

#define MAX_LEN_SNARF_ADAPTER_NAME 64 /* deprecated */
#define EC_SNARF_ADAPTER_NAME_MAXLEN 63
#define EC_SNARF_ADAPTER_NAME_SIZE (EC_SNARF_ADAPTER_NAME_MAXLEN + 1)

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_LINK_PARMS_SNARF
{
    EC_T_LINK_PARMS linkParms;                                      /**< Common link parameters. Signature must be set to EC_LINK_PARMS_SIGNATURE_SNARF */

    EC_T_CHAR       szAdapterName[EC_SNARF_ADAPTER_NAME_SIZE];      /**< SNARF adapter name (zero terminated) */

    /* New parameters in version 2 */
    EC_T_DWORD      dwRxBuffers;                                    /**< Receive buffer count, only used in RTP context, 0: default to 20  */
} EC_PACKED_API EC_T_LINK_PARMS_SNARF;
#include EC_PACKED_INCLUDESTOP

/* Intel Pro 100 family */
/* ==================== */
#define EC_LINK_PARMS_SIGNATURE_I8255X_PATTERN (EC_T_DWORD)0x0000CA20
#define EC_LINK_PARMS_SIGNATURE_I8255X_VERSION (EC_T_DWORD)0x00000001
#define EC_LINK_PARMS_SIGNATURE_I8255X (EC_T_DWORD)(EC_LINK_PARMS_SIGNATURE|EC_LINK_PARMS_SIGNATURE_I8255X_PATTERN|EC_LINK_PARMS_SIGNATURE_I8255X_VERSION)
#define EC_LINK_PARMS_IDENT_I8255X "I8255x"

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_LINK_PARMS_I8255X
{
    EC_T_LINK_PARMS linkParms;      /**< Common link parameters. Signature must be set to EC_LINK_PARMS_SIGNATURE_I8255X */
} EC_PACKED_API EC_T_LINK_PARMS_I8255X;
#include EC_PACKED_INCLUDESTOP

/* Intel Pro 1000 family */
/* ==================== */
#define EC_LINK_PARMS_SIGNATURE_I8254X_PATTERN (EC_T_DWORD)0x0000CA30
#define EC_LINK_PARMS_SIGNATURE_I8254X_VERSION (EC_T_DWORD)0x00000002
#define EC_LINK_PARMS_SIGNATURE_I8254X (EC_T_DWORD)(EC_LINK_PARMS_SIGNATURE|EC_LINK_PARMS_SIGNATURE_I8254X_PATTERN|EC_LINK_PARMS_SIGNATURE_I8254X_VERSION)
#define EC_LINK_PARMS_IDENT_I8254X "I8254x"

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_LINK_PARMS_I8254X
{
    EC_T_LINK_PARMS linkParms;              /**< Common link parameters. Signature must be set to EC_LINK_PARMS_SIGNATURE_I8254X */

    EC_T_WORD       wRxBufferCnt;           /**< Receive buffer count, 0: default to 96 */
    EC_T_WORD       wRxBufferSize;          /**< Recevie buffer size for a single Ethernet frame. 0: buffer optimized for standard Ethernet frame. */
    EC_T_WORD       wTxBufferCnt;           /**< Transmit buffer count, 0: default to 96 */
    EC_T_WORD       wTxBufferSize;          /**< Transmit buffer size for a single Ethernet frame. 0: buffer optimized for standard Ethernet frame. */
    EC_T_BOOL       bDisableLocks;          /**< Disable locks */
    EC_T_DWORD      dwAutoNegTimeout;       /**< Timeout [ms] for auto negotiation */

    EC_T_BOOL       bNotUseDmaBuffers;      /**< Use buffers from DMA (EC_FALSE) or from heap for receive. AllocSend is not supported, when EC_TRUE. */
} EC_PACKED_API EC_T_LINK_PARMS_I8254X;
#include EC_PACKED_INCLUDESTOP

/* Realtek RTL8139      */
/* ==================== */
#define EC_LINK_PARMS_SIGNATURE_RTL8139_PATTERN (EC_T_DWORD)0x0000CA40
#define EC_LINK_PARMS_SIGNATURE_RTL8139_VERSION (EC_T_DWORD)0x00000001
#define EC_LINK_PARMS_SIGNATURE_RTL8139 (EC_T_DWORD)(EC_LINK_PARMS_SIGNATURE|EC_LINK_PARMS_SIGNATURE_RTL8139_PATTERN|EC_LINK_PARMS_SIGNATURE_RTL8139_VERSION)
#define EC_LINK_PARMS_IDENT_RTL8139 "RTL8139"

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_LINK_PARMS_RTL8139
{
    EC_T_LINK_PARMS linkParms;   /**< Common link parameters. Signature must be set to EC_LINK_PARMS_SIGNATURE_RTL8139 */

    EC_T_DWORD      dwRxBuffers; /**< Receive buffer count */
    EC_T_DWORD      dwTxBuffers; /**< Transmit buffer count */
} EC_PACKED_API EC_T_LINK_PARMS_RTL8139;
#include EC_PACKED_INCLUDESTOP

/* Realtek RTL8169      */
/* ==================== */
#define EC_LINK_PARMS_SIGNATURE_RTL8169_PATTERN (EC_T_DWORD)0x0000CA50
#define EC_LINK_PARMS_SIGNATURE_RTL8169_VERSION (EC_T_DWORD)0x00000003
#define EC_LINK_PARMS_SIGNATURE_RTL8169 (EC_T_DWORD)(EC_LINK_PARMS_SIGNATURE|EC_LINK_PARMS_SIGNATURE_RTL8169_PATTERN|EC_LINK_PARMS_SIGNATURE_RTL8169_VERSION)
#define EC_LINK_PARMS_IDENT_RTL8169 "RTL8169"

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_LINK_PARMS_RTL8169
{
    EC_T_LINK_PARMS linkParms;              /**< Common link parameters. Signature must be set to EC_LINK_PARMS_SIGNATURE_RTL8169 */

    EC_T_BOOL       bNotUseDmaBuffers;      /**< EC_TRUE: copy buffer before processing, EC_FALSE: Use buffers from DMA (default) */
    EC_T_BOOL       bAckErrInIrq;           /**< Acknowladge errors in interrupt handler */

    /* New parameters in version 2 */
    EC_T_DWORD      dwRxBuffers;            /**< Receive buffer count. Must be a power of 2, maximum 1024 */
    EC_T_DWORD      dwTxBuffers;            /**< Transmit buffer count. Must be a power of 2, maximum 1024 */

    /* New parameters in version 3 */
    EC_T_BOOL       bNoPhyAccess;           /**< Don't use Mdio to set up the PHY */
} EC_PACKED_API EC_T_LINK_PARMS_RTL8169;
#include EC_PACKED_INCLUDESTOP

/* FreeScale FEC */
/* ============= */
#define EC_LINK_PARMS_SIGNATURE_FSLFEC_PATTERN (EC_T_DWORD)0x0000CA60
#define EC_LINK_PARMS_SIGNATURE_FSLFEC_VERSION  (EC_T_DWORD)0x00000002
#define EC_LINK_PARMS_SIGNATURE_FSLFEC    (EC_T_DWORD)(EC_LINK_PARMS_SIGNATURE|EC_LINK_PARMS_SIGNATURE_FSLFEC_PATTERN|EC_LINK_PARMS_SIGNATURE_FSLFEC_VERSION)
#define EC_LINK_PARMS_IDENT_FSLFEC "FslFec"

typedef enum _EC_T_FEC_TYPE
{
    eFEC_IMX25      = 0, /**< MAC on Freescale i.MX25 (ARM9; ARMv5) */
    eFEC_IMX28      = 1, /**< MAC on Freescale i.MX28 (ARM9; ARMv5) */
    eFEC_IMX53      = 2, /**< MAC on Freescale i.MX53 (ARM Cortex-A8; ARMv7-a) */
    eFEC_IMX6       = 3, /**< MAC on Freescale i.MX6  (ARM Cortex-A9 Single/Dual/Quad; ARMv7-a) */
    eFEC_VF6        = 4, /**< MAC on Freescale VYBRID VF6xx (ARM Cortex-A5 + Cortex-M4) */
    eFEC_IMX7       = 5, /**< MAC on Freescale i.MX7  (ARM Cortex-A9 Single/Dual/Quad; ARMv7-a) */
    eFEC_IMX8       = 6, /**< MAC on Freescale i.MX8  (ARM Cortex-A72/A53 Single/Dual/Quad; ARMv8-a) */
    eFEC_IMX8M      = 7, /**< MAC on Freescale i.MX8M (ARM Cortex-A53 Single/Dual/Quad; ARMv8-a) */
    eFEC_IMXRT1064  = 8, /**< MAC on NXP i.MX RT 1064 (ARM Cortex-M7 ) */

    /* Borland C++ datatype alignment correction */
    eFEC_BCppDummy = 0xFFFFFFFF
} EC_T_FEC_TYPE;

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_LINK_PARMS_FSLFEC
{
    EC_T_LINK_PARMS     linkParms;          /**< Common link parameters. Signature must be set to EC_LINK_PARMS_SIGNATURE_FSLFEC */

    EC_T_DWORD          dwRxBuffers;        /**< Receive buffer count */
    EC_T_DWORD          dwTxBuffers;        /**< Transmit buffer count */

    EC_T_FEC_TYPE       eFecType;           /**< System on Chip type */
    EC_T_PHYINTERFACE   ePhyInterface;      /**< PHY interface type */

    EC_T_BOOL           bUseDmaBuffers;     /**< Use buffers from DMA (EC_TRUE) or from heap for receive and AllocSend not supported (EC_FALSE) */
    EC_T_DWORD          dwPhyAddr;          /**< PHY Address */

    EC_T_BOOL           bNoPinMuxing;       /**< No clock configuration and pin muxing */
    EC_T_BOOL           bDontReadMacAddr;   /**< Read of MAC address disabled */

    /* New parameters in version 2 */
    EC_T_DWORD          dwRxInterrupt;      /**< Receive interrupt number (IRQ) */
} EC_PACKED_API EC_T_LINK_PARMS_FSLFEC;
#include EC_PACKED_INCLUDESTOP

/* FreeScale TSEC/eTSEC */
/* ==================== */
#define EC_LINK_PARMS_SIGNATURE_ETSEC_PATTERN (EC_T_DWORD)0x0000CA70
#define EC_LINK_PARMS_SIGNATURE_ETSEC_VERSION (EC_T_DWORD)0x00000001
#define EC_LINK_PARMS_SIGNATURE_ETSEC (EC_T_DWORD)(EC_LINK_PARMS_SIGNATURE|EC_LINK_PARMS_SIGNATURE_ETSEC_PATTERN|EC_LINK_PARMS_SIGNATURE_ETSEC_VERSION)
#define EC_LINK_PARMS_IDENT_ETSEC "ETSEC"

typedef enum _EC_T_ETSEC_TYPE
{
    eETSEC_P2020RDB         = 0,    /**< MAC on Freescale P2020 */
    eETSEC_TWRP1025         = 1,    /**< MAC on Freescale TWRP1025 */
    eETSEC_ISTMPC8548       = 2,    /**< MAC on Freescale ISTMPC8548 */
    eETSEC_XJ_EPU20C        = 3,    /**< MAC on Freescale XJ EPU20C */
    eETSEC_TWRLS1021A       = 4,    /**< MAC on Freescale TWRLS1021A */
    eETSEC_TQMLS_LS102XA    = 5     /**< MAC on Freescale TQMLS LS102XA */
} EC_T_ETSEC_TYPE;

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_LINK_PARMS_ETSEC
{
    EC_T_LINK_PARMS             linkParms;              /**< Common link parameters. Signature must be set to EC_LINK_PARMS_SIGNATURE_ETSEC */

    EC_T_DWORD                  dwRegisterBase;         /**< Physical base address of register block (4k) */
    EC_T_DWORD                  dwLocalMdioBase;        /**< Physical base address of local MDIO register block (4k). For the eTSEC V1 or TSEC this is the same as dwRegisterBase, for the eTSEC V2 it's not. */
    EC_T_DWORD                  dwPhyMdioBase;          /**< Physical base address of MDIO register block (4k). This is the MDIO base of the (e)TSEC where the PHY (MII bus) is physically connected to (MII interface is shared by (e)TSEC's). */

#define ETSEC_FIXED_LINK  0xFFFFFFFF

    EC_T_DWORD                  dwPhyAddr;              /**< PHY address on MII bus. ETSEC_FIXED_LINK if fixed link configuration */
    EC_T_DWORD                  dwTbiPhyAddr;           /**< Address of internal TBI phy. Any address from [0..31] can be used here, but the address shouldn't collide with any external PHY connected to the external MII bus */

#define ETSEC_LINKFLAG_LINKOK                (1 << 0)
#define ETSEC_LINKFLAG_1000baseT_Full        (1 << 1)
#define ETSEC_LINKFLAG_1000baseT_Half        (1 << 2)
#define ETSEC_LINKFLAG_100baseT_Full         (1 << 3)
#define ETSEC_LINKFLAG_100baseT_Half         (1 << 4)
#define ETSEC_LINKFLAG_10baseT_Full          (1 << 5)
#define ETSEC_LINKFLAG_10baseT_Half          (1 << 6)

    EC_T_DWORD                  dwFixedLinkVal;         /**< Only evaluated if dwPhyAddr == FIXED_LINK. Set to one of the ETSEC_LINKFLAG_* macros. I.e. ETSEC_LINKFLAG_1000baseT_Full */
    EC_T_BYTE                   abyStationAddress[6];   /**< MAC address */
    EC_T_BYTE                   byRes[2];
    EC_T_VOID*                  oMiiBusMtx;             /**< This mutex protect the access to the (shared) MII bus. Set to 0 if mutex shouldn't be used. The MII bus is shared between eTSEC instances. So this mutex should be created once and assigned here for all Linklayer instances */

    /* Interrupt handling */
    EC_T_DWORD                  dwRxInterrupt;          /**< Receive interrupt number (IRQ) */

    EC_T_BOOL                   bNotUseDmaBuffers;      /**< EC_TRUE: copy buffer before processing, EC_FALSE: Use buffers from DMA (default) */

    EC_T_ETSEC_TYPE             eETSECType;             /**< System on Chip type */
    EC_T_BOOL                   bMaster;                /**< Full control over the MAC and need to initialize MAC and the connections to the PHYs */
} EC_PACKED_API EC_T_LINK_PARMS_ETSEC;
#include EC_PACKED_INCLUDESTOP

/* NDISUIO Windows CE   */
/* ==================== */
#define EC_LINK_PARMS_SIGNATURE_NDISUIO_PATTERN (EC_T_DWORD)0x0000CA80
#define EC_LINK_PARMS_SIGNATURE_NDISUIO_VERSION (EC_T_DWORD)0x00000001
#define EC_LINK_PARMS_SIGNATURE_NDISUIO (EC_T_DWORD)(EC_LINK_PARMS_SIGNATURE|EC_LINK_PARMS_SIGNATURE_NDISUIO_PATTERN|EC_LINK_PARMS_SIGNATURE_NDISUIO_VERSION)
#define EC_LINK_PARMS_IDENT_NDISUIO "NdisUio"

#define ECAT_NDISUIO_DEVNAME    TEXT("ECT1:")
#define MAX_LEN_NDISUIO_ADAPTER_NAME ((EC_T_DWORD)64) /* deprecated */
#define EC_NDISUIO_ADAPTER_NAME_MAXLEN 63
#define EC_NDISUIO_ADAPTER_NAME_SIZE (EC_NDISUIO_ADAPTER_NAME_MAXLEN + 1)

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_LINK_PARMS_NDISUIO
{
    EC_T_LINK_PARMS linkParms;      /**< Common link parameters. Signature must be set to EC_LINK_PARMS_SIGNATURE_NDISUIO */

    EC_T_WCHAR      szNetworkAdapterName[EC_NDISUIO_ADAPTER_NAME_SIZE];     /**< Network adapter name (zero terminated) */
} EC_PACKED_API EC_T_LINK_PARMS_NDISUIO;
#include EC_PACKED_INCLUDESTOP


/* Beckhoff CCAT */
/* =============== */
#define EC_LINK_PARMS_SIGNATURE_CCAT_PATTERN (EC_T_DWORD)0x0000CAB0
#define EC_LINK_PARMS_SIGNATURE_CCAT_VERSION (EC_T_DWORD)0x00000002
#define EC_LINK_PARMS_SIGNATURE_CCAT (EC_T_DWORD)(EC_LINK_PARMS_SIGNATURE|EC_LINK_PARMS_SIGNATURE_CCAT_PATTERN|EC_LINK_PARMS_SIGNATURE_CCAT_VERSION)
#define EC_LINK_PARMS_IDENT_CCAT "CCAT"

typedef enum _EC_T_CCAT_TYPE
{
    eCCAT_PCI    = 0, /**< CCAT connected to PCI bus */
    eCCAT_EIM    = 1, /**< CCAT connected via EIM. Used in ARM systems, no DMA */

    /* Borland C++ datatype alignment correction */
    eCCAT_BCppDummy = 0xFFFFFFFF
} EC_T_CCAT_TYPE;

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_LINK_PARMS_CCAT
{
    EC_T_LINK_PARMS linkParms;          /**< Common link parameters. Signature must be set to EC_LINK_PARMS_SIGNATURE_CCAT */

    EC_T_CCAT_TYPE  eCcatType;          /**< CCAT connection type */

    EC_T_UINT64     qwCcatBase;         /**< Physical address of register block, only for eCCAT_EIM */
    EC_T_DWORD      dwCcatSize;         /**< Size of register block, only for eCCAT_EIM */

    EC_T_DWORD      dwRxBufferCnt;      /**< Receive buffer count, only for eCCAT_EIM */
    EC_T_DWORD      dwTxBufferCnt;      /**< Transmit buffer count, only for eCCAT_EIM */
} EC_PACKED_API EC_T_LINK_PARMS_CCAT;
#include EC_PACKED_INCLUDESTOP

/* Texas Instruments CPSW */
/* ====================== */
#define EC_LINK_PARMS_SIGNATURE_CPSW_PATTERN (EC_T_DWORD)0x0000CAC0
#define EC_LINK_PARMS_SIGNATURE_CPSW_VERSION (EC_T_DWORD)0x00000001
#define EC_LINK_PARMS_SIGNATURE_CPSW (EC_T_DWORD)(EC_LINK_PARMS_SIGNATURE|EC_LINK_PARMS_SIGNATURE_CPSW_PATTERN|EC_LINK_PARMS_SIGNATURE_CPSW_VERSION)
#define EC_LINK_PARMS_IDENT_CPSW "CPSW"

typedef enum _EC_T_CPSW_TYPE
{
    eCPSW_AM33XX    = 0, /**< TI AM33xx (e.g. Beaglebone) */
    eCPSW_AM387X    = 1, /**< TI DM814x/AM387x (e.g. Mistral/TI 814X/387X BASE EVM) */
    eCPSW_AM437X    = 2, /**< TI AM437x */
    eCPSW_AM57X     = 3, /**< TI AM57x */

    /* Borland C++ datatype alignment correction */
    eCPSW_BCppDummy = 0xFFFFFFFF
} EC_T_CPSW_TYPE;


typedef struct _EC_T_LINK_PARMS_CPSW
{
    EC_T_LINK_PARMS linkParms;                              /**< Common link parameters. Signature must be set to EC_LINK_PARMS_SIGNATURE_CPSW */

    EC_T_CPSW_TYPE              eCpswType;                  /**< CPSW type */
    EC_T_DWORD                  dwPhyAddr;                  /**< PHY address */
    EC_T_DWORD                  dwPortPrio;                 /**< 0 (lowest), 1 (highest) */
    EC_T_BOOL                   bMaster;                    /**< EC_TRUE: Initialize MAC */
    EC_T_BOOL                   bPhyRestartAutoNegotiation; /**< EC_TRUE: Restart auto negotiation on initialization */
    EC_T_PHYINTERFACE           ePhyInterface;              /**< PHY connection type */

    /* Interrupt handling */
    EC_T_DWORD                  dwRxInterrupt;              /**< Receive interrupt number (IRQ) */
    EC_T_BOOL                   bNotUseDmaBuffers;          /**< Use buffers from DMA (EC_FALSE) or from heap for receive. AllocSend is not supported, when EC_TRUE */
} EC_T_LINK_PARMS_CPSW;


/* Xilinx GEM */
/* ====================== */
#define EC_LINK_PARMS_SIGNATURE_GEM_PATTERN (EC_T_DWORD)0x0000CAD0
#define EC_LINK_PARMS_SIGNATURE_GEM_VERSION (EC_T_DWORD)0x00000003
#define EC_LINK_PARMS_SIGNATURE_GEM         (EC_T_DWORD)(EC_LINK_PARMS_SIGNATURE|EC_LINK_PARMS_SIGNATURE_GEM_PATTERN|EC_LINK_PARMS_SIGNATURE_GEM_VERSION)
#define EC_LINK_PARMS_IDENT_GEM "GEM"

/* Needed for IOCTL */
#define GEM_LINKFLAG_LINKOK                (1 << 0)
#define GEM_LINKFLAG_1000baseT_Full        (1 << 1)
#define GEM_LINKFLAG_100baseT_Full         (1 << 3)

#define GEM_SLAVECNT 2

/* Select the source of the Rx clock, control and data signals */
typedef enum _EC_T_GEM_RXSOURCE
{
    eGemRxSource_MIO       = 0,     /**< MIO as source for RX clock, control and data signals */
    eGemRxSource_EMIO      = 1,     /**< EMIO as source for RX clock, control and data signals */

    /* Borland C++ datatype alignment correction */
    eGemRxSource_BCppDummy = 0xFFFFFFFF
} EC_T_GEM_RXSOURCE;

typedef enum _EC_T_GEM_TYPE
{
    eGemType_Zynq7000       = 0,    /**< Xilinx Zynq 7000 */
    eGemType_ZynqUltrascale = 1,    /**< Xilinx Zynq Ultrascale */

    /* Borland C++ datatype alignment correction */
    eGemType_BCppDummy      = 0xFFFFFFFF
} EC_T_GEM_TYPE;

typedef enum _EC_T_GEM_CLK_DIV_TYPE
{
    eGemClkDivType_default = 0,    /**< Xilinx Zynq Ultrascale Board ZCU102, ZCU104  */
    eGemClkDivType_K26     = 1,    /**< Xilinx Zynq Ultrascale SOM K26 */

    /* Borland C++ datatype alignment correction */
    eGemClkDivType_BCppDummy      = 0xFFFFFFFF
} EC_T_GEM_CLK_DIV_TYPE;

typedef struct _EC_T_LINK_PARMS_GEM
{
    EC_T_LINK_PARMS         linkParms;              /**< Common link parameters. Signature must be set to EC_LINK_PARMS_SIGNATURE_GEM */

    EC_T_GEM_RXSOURCE       eRxSource;              /**< Source of RX clock, control and data signals */
    EC_T_DWORD              dwPhyAddr;              /**< PHY address */

    /* Interrupt handling */
    EC_T_DWORD              dwRxInterrupt;          /**< Receive interrupt number (IRQ) */

    EC_T_BOOL               bUseDmaBuffers;         /**< Use buffers from DMA (EC_TRUE) or from heap for receive. AllocSend is not supported, when EC_FALSE. */
    EC_T_BOOL               bNoPhyAccess;           /**< EC_FALSE: Link layer should initialize PHY and read link status (connected/disconnected). EC_TRUE: Client is responsible of PHY initialization and clock initialization */

    EC_T_BOOL               bUseGmiiToRgmiiConv;    /**< Use XILINX GMIITORGMII Converter (EC_TRUE) */
    EC_T_DWORD              dwConvPhyAddr;          /**< PHY address used to communicate with converter. In Linux doc it named "reg" */

    EC_T_DWORD              dwTxDmaDesCnt;          /**< Transmit DMA descriptor buffer count. Must be a power of 2, maximum 256 */
    EC_T_DWORD              dwRxDmaDesCnt;          /**< Receive DMA descriptor buffer count. Must be a power of 2, maximum 256 */

    EC_T_GEM_TYPE           eGemType;               /**< System on Chip type */

    /* New parameters in version 2 */
    EC_T_PHYINTERFACE       ePhyInterface;          /**< PHY connection type */

    /* New parameters in version 3 */
    EC_T_BOOL               bNoPinMuxing;           /**< No clock configuration and pin muxing */
    EC_T_GEM_CLK_DIV_TYPE   eClkDivType;            /**< Change Ref Clock settings */
} EC_T_LINK_PARMS_GEM;

/* SMSC LAN9218i */
/* ============= */
#define EC_LINK_PARMS_SIGNATURE_L9218I_PATTERN (EC_T_DWORD)0x0000CAE0
#define EC_LINK_PARMS_SIGNATURE_L9218I_VERSION (EC_T_DWORD)0x00000001
#define EC_LINK_PARMS_SIGNATURE_L9218I (EC_T_DWORD)(EC_LINK_PARMS_SIGNATURE|EC_LINK_PARMS_SIGNATURE_L9218I_PATTERN|EC_LINK_PARMS_SIGNATURE_L9218I_VERSION)
#define EC_LINK_PARMS_IDENT_L9218I "L9218i"

typedef struct _EC_T_LINK_PARMS_L9218I
{
    EC_T_LINK_PARMS linkParms;      /**< Common link parameters. Signature must be set to EC_LINK_PARMS_SIGNATURE_L9218I */

    EC_T_DWORD      dwTxBuffers;    /**< Transmit buffer count */
} EC_T_LINK_PARMS_L9218I;


/* Xilinx LogiCORE IP XPS Ethernet Lite Media Access Controller */
/* ============================================================ */
#define EC_LINK_PARMS_SIGNATURE_EMAC_PATTERN (EC_T_DWORD)0x0000CAF0
#define EC_LINK_PARMS_SIGNATURE_EMAC_VERSION (EC_T_DWORD)0x00000001
#define EC_LINK_PARMS_SIGNATURE_EMAC (EC_T_DWORD)(EC_LINK_PARMS_SIGNATURE|EC_LINK_PARMS_SIGNATURE_EMAC_PATTERN|EC_LINK_PARMS_SIGNATURE_EMAC_VERSION)
#define EC_LINK_PARMS_IDENT_EMAC "EMAC"

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_LINK_PARMS_EMAC
{
    EC_T_LINK_PARMS linkParms;          /**< Common link parameters. Signature must be set to EC_LINK_PARMS_SIGNATURE_EMAC */

    EC_T_DWORD      dwRegisterBase;     /**< Physical base address of register block (4k) */
    EC_T_DWORD      dwPhyAddr;          /**< PHY address */
    EC_T_DWORD      dwRxInterrupt;      /**< Receive interrupt number (IRQ) */
    EC_T_DWORD      dwRegisterLength;   /**< Physical length of register block */
    EC_T_BOOL       bNotUseDmaBuffers;  /**< EC_TRUE: copy buffer before processing, EC_FALSE: Use buffers from DMA (default) */
} EC_PACKED_API EC_T_LINK_PARMS_EMAC;
#include EC_PACKED_INCLUDESTOP

/* Intel EG20T MAC */
/* =============== */
#define EC_LINK_PARMS_SIGNATURE_EG20T_PATTERN (EC_T_DWORD)0x0000CB00
#define EC_LINK_PARMS_SIGNATURE_EG20T_VERSION (EC_T_DWORD)0x00000001
#define EC_LINK_PARMS_SIGNATURE_EG20T (EC_T_DWORD)(EC_LINK_PARMS_SIGNATURE|EC_LINK_PARMS_SIGNATURE_EG20T_PATTERN|EC_LINK_PARMS_SIGNATURE_EG20T_VERSION)
#define EC_LINK_PARMS_IDENT_EG20T "EG20T"

/* Needed for IOCTL */
#define EG20T_LINKFLAG_LINKOK                (1 << 0)
#define EG20T_LINKFLAG_1000baseT_Full        (1 << 1)
#define EG20T_LINKFLAG_1000baseT_Half        (1 << 2)
#define EG20T_LINKFLAG_100baseT_Full         (1 << 3)
#define EG20T_LINKFLAG_100baseT_Half         (1 << 4)
#define EG20T_LINKFLAG_10baseT_Full          (1 << 5)
#define EG20T_LINKFLAG_10baseT_Half          (1 << 6)

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_LINK_PARMS_EG20T
{
    EC_T_LINK_PARMS linkParms;      /**< Common link parameters. Signature must be set to EC_LINK_PARMS_SIGNATURE_EG20T */
} EC_PACKED_API EC_T_LINK_PARMS_EG20T;
#include EC_PACKED_INCLUDESTOP

/* Proxy Link Layer */
/* ================ */
#define EC_LINK_PARMS_SIGNATURE_PROXY_PATTERN (EC_T_DWORD)0x00005200
#define EC_LINK_PARMS_SIGNATURE_PROXY_VERSION (EC_T_DWORD)0x00000001
#define EC_LINK_PARMS_SIGNATURE_PROXY (EC_T_DWORD)(EC_LINK_PARMS_SIGNATURE|EC_LINK_PARMS_SIGNATURE_PROXY_PATTERN|EC_LINK_PARMS_SIGNATURE_PROXY_VERSION)
#define EC_LINK_PARMS_IDENT_PROXY "Proxy"

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_LINK_PARMS_PROXY
{
    EC_T_LINK_PARMS linkParms;          /**< Common link parameters. Signature must be set to EC_LINK_PARMS_SIGNATURE_PROXY */

    EC_T_DWORD      dwSocketType;       /**< Socket type. Must be set to 2 (emrassocktype_udp) */
    EC_T_BYTE       abySrcIpAddress[4]; /**< Source adapter IP address (listen) */
    EC_T_WORD       wSrcPort;           /**< Source port number (listen) */
    EC_T_BYTE       abyDstIpAddress[4]; /**< Destination adapter IP address (connect) */
    EC_T_WORD       wDstPort;           /**< Destination port number (connect) */

    EC_T_BYTE       abyMac[6];          /**< MAC address */
    EC_T_DWORD      dwRxBufferCnt;      /**< Frame buffer count for interrupt service thread (IST) */
} EC_PACKED_API EC_T_LINK_PARMS_PROXY;
#include EC_PACKED_INCLUDESTOP

/* PMX1000-R6040 */
/* ============= */
#define EC_LINK_PARMS_SIGNATURE_R6040_PATTERN (EC_T_DWORD)0x0000CB10
#define EC_LINK_PARMS_SIGNATURE_R6040_VERSION (EC_T_DWORD)0x00000001
#define EC_LINK_PARMS_SIGNATURE_R6040 (EC_T_DWORD)(EC_LINK_PARMS_SIGNATURE|EC_LINK_PARMS_SIGNATURE_R6040_PATTERN|EC_LINK_PARMS_SIGNATURE_R6040_VERSION)
#define EC_LINK_PARMS_IDENT_R6040 "R6040"

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_LINK_PARMS_R6040
{
    EC_T_LINK_PARMS linkParms;      /**< Common link parameters. Signature must be set to EC_LINK_PARMS_SIGNATURE_R6040 */

    EC_T_DWORD      dwTxDmaDesCnt;  /**< Transmit DMA descriptor buffer count. Must be a power of 2, maximum 256 */
    EC_T_DWORD      dwRxDmaDesCnt;  /**< Receive DMA descriptor buffer count. Must be a power of 2, maximum 256 */
} EC_PACKED_API EC_T_LINK_PARMS_R6040;
#include EC_PACKED_INCLUDESTOP

/* SockRaw Linux */
/* ============= */
#define EC_LINK_PARMS_SIGNATURE_SOCKRAW_PATTERN (EC_T_DWORD)0x0000CC10
#define EC_LINK_PARMS_SIGNATURE_SOCKRAW_VERSION (EC_T_DWORD)0x00000004
#define EC_LINK_PARMS_SIGNATURE_SOCKRAW (EC_T_DWORD)(EC_LINK_PARMS_SIGNATURE|EC_LINK_PARMS_SIGNATURE_SOCKRAW_PATTERN|EC_LINK_PARMS_SIGNATURE_SOCKRAW_VERSION)
#define EC_LINK_PARMS_IDENT_SOCKRAW "SockRaw"

#define MAX_LEN_SOCKRAW_ADAPTER_NAME 64 /* deprecated */
#define EC_SOCKRAW_ADAPTER_NAME_MAXLEN 63
#define EC_SOCKRAW_ADAPTER_NAME_SIZE (EC_SOCKRAW_ADAPTER_NAME_MAXLEN + 1)

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_LINK_PARMS_SOCKRAW
{
    EC_T_LINK_PARMS linkParms;                                          /**< Common link parameters. Signature must be set to EC_LINK_PARMS_SIGNATURE_SOCKRAW */

    EC_T_CHAR           szAdapterName[EC_SOCKRAW_ADAPTER_NAME_SIZE];    /**< Native eth device name, e.g. "eth0" (zero terminated) */
    EC_T_BOOL           bDisableForceBroadcast;                         /**< Don't change target MAC address to FF:FF:FF:FF:FF:FF */

    /* New parameters in version 2 */
    EC_T_BOOL           bReplacePaddingWithNopCmd;                      /**< Prevent adding Ethernet padding to work-around EtherCAT corruption bugs from native Linux driver(s). */

    /* New parameters in version 3 */
    EC_T_BOOL           bUsePacketMmapRx;                               /**< Use PACKET_MMAP PACKET_RX_RING for receive */

    /* New parameters in version 4 */
    EC_T_BOOL           bSetCoalescingParms;                            /**< Set Coalescing parms to enhance the linklayer performance */
} EC_PACKED_API EC_T_LINK_PARMS_SOCKRAW;
#include EC_PACKED_INCLUDESTOP

/*****************************************************************************/
/* NIOS                                                                      */
/*****************************************************************************/
#define EC_LINK_PARMS_SIGNATURE_NIOS_PATTERN (EC_T_DWORD)0x0000CB20
#define EC_LINK_PARMS_SIGNATURE_NIOS_VERSION (EC_T_DWORD)0x00000001
#define EC_LINK_PARMS_SIGNATURE_NIOS (EC_T_DWORD)(EC_LINK_PARMS_SIGNATURE|EC_LINK_PARMS_SIGNATURE_NIOS_PATTERN|EC_LINK_PARMS_SIGNATURE_NIOS_VERSION)
#define EC_LINK_PARMS_IDENT_NIOS "NIOS"

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_LINK_PARMS_NIOS
{
    EC_T_LINK_PARMS linkParms;      /**< Common link parameters. Signature must be set to EC_LINK_PARMS_SIGNATURE_NIOS */
} EC_PACKED_API EC_T_LINK_PARMS_NIOS;
#include EC_PACKED_INCLUDESTOP

/*****************************************************************************/
/* RIN32                                                                     */
/*****************************************************************************/
#define EC_LINK_PARMS_SIGNATURE_RIN32_PATTERN (EC_T_DWORD)0x0000CD10
#define EC_LINK_PARMS_SIGNATURE_RIN32_VERSION (EC_T_DWORD)0x00000001
#define EC_LINK_PARMS_SIGNATURE_RIN32 (EC_T_DWORD)(EC_LINK_PARMS_SIGNATURE|EC_LINK_PARMS_SIGNATURE_RIN32_PATTERN|EC_LINK_PARMS_SIGNATURE_RIN32_VERSION)
#define EC_LINK_PARMS_IDENT_RIN32 "RIN32"

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_LINK_PARMS_RIN32
{
    EC_T_LINK_PARMS linkParms;      /**< Common link parameters. Signature must be set to EC_LINK_PARMS_SIGNATURE_RIN32 */
} EC_PACKED_API EC_T_LINK_PARMS_RIN32;
#include EC_PACKED_INCLUDESTOP

/*****************************************************************************/
/* DW3504                                                                      */
/*****************************************************************************/
#define EC_LINK_PARMS_SIGNATURE_DW3504_PATTERN (EC_T_DWORD)0x0000CE10
#define EC_LINK_PARMS_SIGNATURE_DW3504_VERSION (EC_T_DWORD)0x00000001
#define EC_LINK_PARMS_SIGNATURE_DW3504    (EC_T_DWORD)(EC_LINK_PARMS_SIGNATURE|EC_LINK_PARMS_SIGNATURE_DW3504_PATTERN|EC_LINK_PARMS_SIGNATURE_DW3504_VERSION)
#define EC_LINK_PARMS_IDENT_DW3504 "DW3504"

#include EC_PACKED_API_INCLUDESTART
typedef enum _EC_T_DW3504_TYPE
{
    eDW3504_CycloneV = 0, /**< MAC on Cyclone V SoC */
    eDW3504_LCES1    = 1, /**< MAC on LCES1 SoC     */
    eDW3504_RZN1     = 2, /**< MAC on Renesas RZN1  */
    eDW3504_STM32MP1 = 3, /**< MAC on STM32MP1      */
    eDW3504_ATOM     = 4, /**< MAC on Atom 6000     */

    /* datatype alignment correction */
    eDW3504_BCppDummy = 0xFFFFFFFF
} EC_T_DW3504_TYPE;
#include EC_PACKED_INCLUDESTOP

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_LINK_PARMS_DW3504
{
    EC_T_LINK_PARMS   linkParms;              /**< Common link parameters. Signature must be set to EC_LINK_PARMS_SIGNATURE_DW3504 */

    EC_T_DWORD        dwPhyAddr;              /**< PHY address */
    EC_T_DWORD        dwRegisterBasePhys;     /**< Physical base address of register block (8k) */
    EC_T_DW3504_TYPE  eDW3504Type;            /**< System on Chip type */
    EC_T_PHYINTERFACE ePhyInterface;          /**< PHY connection type */
    EC_T_BOOL         bNotUseDmaBuffers;      /**< Use buffers from DMA (EC_FALSE) or from heap for receive. AllocSend is not supported, when EC_TRUE */

    EC_T_DWORD        dwTxDmaDesCnt;          /**< Transmit DMA descriptor buffer count. Must be a power of 2, maximum 256 */
    EC_T_DWORD        dwRxDmaDesCnt;          /**< Receive DMA descriptor buffer count. Must be a power of 2, maximum 256 */
} EC_PACKED_API EC_T_LINK_PARMS_DW3504;
#include EC_PACKED_INCLUDESTOP

/*****************************************************************************/
/* RZT1                                                                      */
/*****************************************************************************/
#define EC_LINK_PARMS_SIGNATURE_RZT1_PATTERN (EC_T_DWORD)0x0000CD10
#define EC_LINK_PARMS_SIGNATURE_RZT1_VERSION (EC_T_DWORD)0x00000001
#define EC_LINK_PARMS_SIGNATURE_RZT1 (EC_T_DWORD)(EC_LINK_PARMS_SIGNATURE|EC_LINK_PARMS_SIGNATURE_RZT1_PATTERN|EC_LINK_PARMS_SIGNATURE_RZT1_VERSION)
#define EC_LINK_PARMS_IDENT_RZT1 "RZT1"

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_LINK_PARMS_RZT1
{
    EC_T_LINK_PARMS linkParms;      /**< Common link parameters. Signature must be set to EC_LINK_PARMS_SIGNATURE_RZT1 */
} EC_PACKED_API EC_T_LINK_PARMS_RZT1;
#include EC_PACKED_INCLUDESTOP

/*****************************************************************************/
/* ICSS - Texas Instruments industrial communications subsystem (PRU ICSS)   */
/*****************************************************************************/
#define EC_LINK_PARMS_SIGNATURE_ICSS_PATTERN (EC_T_DWORD)0x0000CB30
#define EC_LINK_PARMS_SIGNATURE_ICSS_VERSION (EC_T_DWORD)0x00000006
#define EC_LINK_PARMS_SIGNATURE_ICSS (EC_T_DWORD)(EC_LINK_PARMS_SIGNATURE|EC_LINK_PARMS_SIGNATURE_ICSS_PATTERN|EC_LINK_PARMS_SIGNATURE_ICSS_VERSION)
#define EC_LINK_PARMS_IDENT_ICSS "ICSS"

typedef enum _EC_T_LINK_ICSS_BOARD
{
    EcLinkIcssBoard_Unsupported = 0,
    EcLinkIcssBoard_am572x = 1,         /**< TI AM572x */
    EcLinkIcssBoard_am571x = 2,         /**< TI AM571x */
    EcLinkIcssBoard_am3359 = 3,         /**< TI AM3359 */
    EcLinkIcssBoard_am572x_emerson = 4, /**< TI AM572x on Emerson board */
    EcLinkIcssBoard_am574x = 5          /**< TI AM574x */

    /* Borland C++ datatype alignment correction
    EcLinkIcssBoard_BCppDummy   = 0xFFFFFFFF */
} EC_T_LINK_ICSS_BOARD;

typedef EC_T_VOID (*EC_T_LINK_ICSS_TTS_CALLBACK)(EC_T_VOID*);

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_LINK_PARMS_ICSS
{
    EC_T_LINK_PARMS      linkParms;         /**< Common link parameters. Signature must be set to EC_LINK_PARMS_SIGNATURE_ICSS */

    EC_T_LINK_ICSS_BOARD eBoardType;        /**< TI System on Chip board type */
    EC_T_BOOL            bMaster;           /**< Initialize whole PRUSS subsystem, not only port. This flag is always required when link layer is used on single ICSS port. This flag is also required, when link layer is used in "Redundancy mode" und two ICSS ports are used. In this case, first port should be master, and second port should be slave */
    EC_T_BOOL            bNoMacAddr;        /**< EC_TRUE: No MAC address registers access */

    /* New parameters in version 2 */
    EC_T_LINK_TTS        TtsParms;          /**< Time Triggered Send parameters */

    /* New parameters in version 3 */
    EC_T_DWORD           dwPhyAddr;         /**< PHY address */
    EC_T_PHYINTERFACE    ePhyInterface;     /**< PHY connection type */

    /* New parameters in version 4 */
    EC_T_BOOL            bNoPhyReset;       /**< No hardware reset of the PHY */

    /* New parameters in version 5 */
    EC_T_BOOL            bUseAllSendQueues; /**< Use the additional 3 queues with lower priority to send more frames per cycle */

    /* New parameters in version 6 */
    EC_T_BOOL            bLegacyFirmware;   /**< For am57xx use legacy ICSS firmware from pdk_am57xx_1_0_6, instead of pdk_am57xx_1_0_17 with patch for Rx error issue, see https://e2e.ti.com/support/processors-group/processors/f/processors-forum/1022410/am5746-rx_error_offset-conditions/3788558#3788558 */
} EC_PACKED_API EC_T_LINK_PARMS_ICSS;
#include EC_PACKED_INCLUDESTOP

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_LINK_ICSS_STATISTICS
{
    EC_T_DWORD           dwTxBroadcastFramesCnt;
    EC_T_DWORD           dwTxMulticastFramesCnt;
    EC_T_DWORD           dwTxUnicastFramesCnt;
    EC_T_DWORD           dwTxByteCnt;
    EC_T_DWORD           dwRxBroadcastFramesCnt;
    EC_T_DWORD           dwRxMulticastFramesCnt;
    EC_T_DWORD           dwRxUnicastFramesCnt;
    EC_T_DWORD           dwRxByteCnt;
    EC_T_DWORD           dwTx64ByteFramesCnt;
    EC_T_DWORD           dwTx65_127ByteFramesCnt;
    EC_T_DWORD           dwTx128_255ByteFramesCnt;
    EC_T_DWORD           dwTx256_511ByteFramesCnt;
    EC_T_DWORD           dwTx512_1023Byte_FramesCnt;
    EC_T_DWORD           dwTxGreaterThan1024ByteFramesCnt;
    EC_T_DWORD           dwRx64ByteFramesCnt;
    EC_T_DWORD           dwRx65_127ByteFramesCnt;
    EC_T_DWORD           dwRx128_255ByteFramesCnt;
    EC_T_DWORD           dwRx256_511ByteFramesCnt;
    EC_T_DWORD           dwRx512_1023Byte_FramesCnt;
    EC_T_DWORD           dwRxGreaterThan1024ByteFramesCnt;
    EC_T_DWORD           dwLateCollisionFramesCnt;
    EC_T_DWORD           dwSingleCollisionBytesCnt;
    EC_T_DWORD           dwMultipleCollisionBytesCnt;
    EC_T_DWORD           dwExcessCollisionBytesCnt;
    EC_T_DWORD           dwRxMisalignmentFramesCnt;
    EC_T_DWORD           dwStormPreventionFramesCnt;
    EC_T_DWORD           dwRxErrorFramesCnt;
    EC_T_DWORD           dwSfdErrorFramesCnt;
    EC_T_DWORD           dwTxDeferredFramesCnt;
    EC_T_DWORD           dwRxOversizedFramesCnt;
    EC_T_DWORD           dwRxUndersizedFramesCnt;
    EC_T_DWORD           dwRxCrcErrorFramesCnt;
    EC_T_DWORD           dwRxDroppedFramesCnt;
    EC_T_DWORD           dwTxFifoOverflowCnt;
    EC_T_DWORD           dwTxFifoUnderflowCnt;
} EC_PACKED_API EC_T_LINK_ICSS_STATISTICS;
#include EC_PACKED_INCLUDESTOP

/**************************************************************************************/
/* ICSSG - Texas Instruments Gigabit industrial communications subsystem  (PRU ICSSG) */
/**************************************************************************************/
#define EC_LINK_PARMS_SIGNATURE_ICSSG_PATTERN (EC_T_DWORD)0x0000CB90
#define EC_LINK_PARMS_SIGNATURE_ICSSG_VERSION (EC_T_DWORD)0x00000001
#define EC_LINK_PARMS_SIGNATURE_ICSSG (EC_T_DWORD)(EC_LINK_PARMS_SIGNATURE|EC_LINK_PARMS_SIGNATURE_ICSSG_PATTERN|EC_LINK_PARMS_SIGNATURE_ICSSG_VERSION)
#define EC_LINK_PARMS_IDENT_ICSSG "ICSSG"

typedef enum _EC_T_LINK_ICSSG_BOARD
{
    EcLinkIcssgBoard_Unsupported = 0,
    EcLinkIcssgBoard_am654x = 1,         /**< TI AM654x */

    /* Borland C++ datatype alignment correction */
    EcLinkIcssgBoard_BCppDummy = 0xFFFFFFFF
} EC_T_LINK_ICSSG_BOARD;

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_LINK_PARMS_ICSSG
{
    EC_T_LINK_PARMS       linkParms;        /**< Common link parameters. Signature must be set to EC_LINK_PARMS_SIGNATURE_ICSSG */
    EC_T_LINK_ICSSG_BOARD eBoardType;       /**< TI System on Chip board type */
    EC_T_BOOL             bMaster;          /**< Initialize whole PRUSS subsystem, not only port. This flag is always required when link layer is used on single ICSSG port. This flag is also required, when link layer is used in "Redundancy mode" und two ICSSG ports are used. In this case, first port should be master, and second port should be slave */
} EC_PACKED_API EC_T_LINK_PARMS_ICSSG;
#include EC_PACKED_INCLUDESTOP

/*****************************************************************************/
/* AlteraTSE - Intel FPGA Triple-Speed Ethernet IP Core                      */
/*****************************************************************************/
#define EC_LINK_PARMS_SIGNATURE_ALTERATSE_PATTERN (EC_T_DWORD)0x0000DE30
#define EC_LINK_PARMS_SIGNATURE_ALTERATSE_VERSION (EC_T_DWORD)0x00000001
#define EC_LINK_PARMS_SIGNATURE_ALTERATSE (EC_T_DWORD)(EC_LINK_PARMS_SIGNATURE|EC_LINK_PARMS_SIGNATURE_ALTERATSE_PATTERN|EC_LINK_PARMS_SIGNATURE_ALTERATSE_VERSION)
#define EC_LINK_PARMS_IDENT_ALTERATSE "AlteraTSE"

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_LINK_PARMS_ALTERATSE
{
    EC_T_LINK_PARMS      linkParms;     /**< Common link parameters. Signature must be set to EC_LINK_PARMS_SIGNATURE_ALTERATSE */

    EC_T_BYTE            abyMac[6];     /**< MAC address */
} EC_PACKED_API EC_T_LINK_PARMS_ALTERATSE;
#include EC_PACKED_INCLUDESTOP

/* Profichip ANTAIOS */
/* ==================== */
#define EC_LINK_PARMS_SIGNATURE_ANTAIOS_PATTERN (EC_T_DWORD)0x0000CB40
#define EC_LINK_PARMS_SIGNATURE_ANTAIOS_VERSION (EC_T_DWORD)0x00000001
#define EC_LINK_PARMS_SIGNATURE_ANTAIOS (EC_T_DWORD)(EC_LINK_PARMS_SIGNATURE|EC_LINK_PARMS_SIGNATURE_ANTAIOS_PATTERN|EC_LINK_PARMS_SIGNATURE_ANTAIOS_VERSION)
#define EC_LINK_PARMS_IDENT_ANTAIOS "ANTAIOS"

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_LINK_PARMS_ANTAIOS
{
    EC_T_LINK_PARMS linkParms;      /**< Common link parameters. Signature must be set to EC_LINK_PARMS_SIGNATURE_ANTAIOS */
} EC_PACKED_API EC_T_LINK_PARMS_ANTAIOS;
#include EC_PACKED_INCLUDESTOP

/*****************************************************************************/
/* SH Eth                                                                    */
/*****************************************************************************/
#define EC_LINK_PARMS_SIGNATURE_SHETH_PATTERN (EC_T_DWORD)0x0000CF10
#define EC_LINK_PARMS_SIGNATURE_SHETH_VERSION (EC_T_DWORD)0x00000001
#define EC_LINK_PARMS_SIGNATURE_SHETH (EC_T_DWORD)(EC_LINK_PARMS_SIGNATURE|EC_LINK_PARMS_SIGNATURE_SHETH_PATTERN|EC_LINK_PARMS_SIGNATURE_SHETH_VERSION)
#define EC_LINK_PARMS_IDENT_SHETH "SHEth"

#define SHETH_FIXED_LINK  0xFF

/* Super H Ethernet controller type */
typedef enum
{
    eSHEth_R8A777X     = 0,     /**< Renesas R8A777X */
    eSHEth_R8A779X     = 1,     /**< Renesas R8A779X */
    eSHEth_SH7724      = 2,     /**< Renesas SH7724 */
    eSHEth_SH7757      = 3,     /**< Renesas SH7757 */
    eSHEth_SH7757_GIGA = 4,     /**< Renesas SH7757_GIGA */
    eSHEth_SH7734      = 5,     /**< Renesas SH7734 */
    eSHEth_SH7763      = 6,     /**< Renesas SH7763 */
    eSHEth_R8A7740     = 7,     /**< Renesas R8A7740 */
    eSHEth_R7S72100    = 8,     /**< Renesas R7S72100 */
    eSHEth_SH7619      = 9,     /**< Renesas SH7619 */
    eSHEth_SH771X      = 10,    /**< Renesas SH771X */
    eSHEth_R8A77400    = 11,    /**< Renesas R8A77400 */
    eSHEth_R8A77435    = 12,    /**< Renesas R8A77435 */
    eSHEth_R8A77430    = 13,    /**< Renesas R8A77430 */
    eSHEth_R8A77450    = 14,    /**< Renesas R8A77450 */

    /* Borland C++ datatype alignment correction */
    eSHEth_BCppDummy   = 0xFFFFFFFF
} EC_T_SHETH_TYPE;

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_LINK_PARMS_SHETH
{
    EC_T_LINK_PARMS linkParms;              /**< Common link parameters. Signature must be set to EC_LINK_PARMS_SIGNATURE_SHETH */

    EC_T_SHETH_TYPE eType;                  /**< System on Chip type */
    EC_T_BYTE       abyStationAddress[6];   /**< MAC address */
    EC_T_DWORD      dwBaseAddr;             /**< Physical address of register block */
    EC_T_BYTE       byPhyAddr;              /**< PHY address */

    EC_T_BOOL       bNotUseDmaBuffers;      /**< EC_TRUE: copy buffer before processing, EC_FALSE: Use buffers from DMA (default) */
    EC_T_DWORD      dwTxDmaDesCnt;          /**< Transmit DMA descriptor buffer count. Must be a power of 2, maximum 256 */
    EC_T_DWORD      dwRxDmaDesCnt;          /**< Receive DMA descriptor buffer count. Must be a power of 2, maximum 256 */

} EC_PACKED_API EC_T_LINK_PARMS_SHETH;
#include EC_PACKED_INCLUDESTOP

/* EC-Simulator     */
/* ================ */
#define EC_LINK_PARMS_SIGNATURE_SIMULATOR_PATTERN (EC_T_DWORD)0x00005100
#define EC_LINK_PARMS_SIGNATURE_SIMULATOR_VERSION (EC_T_DWORD)0x00000009
#define EC_LINK_PARMS_SIGNATURE_SIMULATOR (EC_T_DWORD)(EC_LINK_PARMS_SIGNATURE|EC_LINK_PARMS_SIGNATURE_SIMULATOR_PATTERN|EC_LINK_PARMS_SIGNATURE_SIMULATOR_VERSION)
#define EC_LINK_PARMS_IDENT_SIMULATOR "Simulator"

#define MAX_LEN_SIMULATOR_ENI_FILE_NAME 256 /* (deprecated) */
#define MAX_LEN_SIMULATOR_KEY 64            /* (deprecated) */

#define EC_SIMULATOR_ENI_FILE_NAME_MAXLEN 255
#define EC_SIMULATOR_ENI_FILE_NAME_SIZE (EC_SIMULATOR_ENI_FILE_NAME_MAXLEN + 1)
#define EC_SIMULATOR_KEY_MAXLEN 255
#define EC_SIMULATOR_KEY_SIZE (EC_SIMULATOR_KEY_MAXLEN + 1)

#define EC_SIMULATOR_DEVICE_CONNECTION_TYPE_AUTO         0
#define EC_SIMULATOR_DEVICE_CONNECTION_TYPE_SLAVE        1
#define EC_SIMULATOR_DEVICE_CONNECTION_TYPE_DEVICE       2
#define EC_SIMULATOR_DEVICE_CONNECTION_TYPE_DISCONNECTED 3

#define EC_SIMULATOR_MAX_LINK_PARMS                      4

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_SIMULATOR_DEVICE_CONNECTION_DESC
{
    EC_T_DWORD dwType;                /* EC_SIMULATOR_DEVICE_CONNECTION_TYPE_... */
    EC_T_DWORD dwInstanceID;          /* EC-Simulator Instance ID */
    EC_T_WORD  wCfgFixedAddress;      /* EC-Simulator Configuration (ENI/EXI) */
    EC_T_BYTE  byPort;                /* 0...3: Port A-D */
} EC_PACKED_API EC_T_SIMULATOR_DEVICE_CONNECTION_DESC;
#include EC_PACKED_INCLUDESTOP

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_LINK_PARMS_SIMULATOR
{
    EC_T_LINK_PARMS linkParms;                                  /**< Common link parameters. Signature must be set to EC_LINK_PARMS_SIGNATURE_SIMULATOR */

    struct _EC_T_OS_PARMS* pOsParms;                            /**< OS layer parameters */

    /* topology parameters */
    EC_T_CNF_TYPE   eCnfType;                                   /**< optional: create slaves from ENI/EXI, see esConfigureNetwork */
    EC_T_BYTE*      pbyCnfData;                                 /**< optional: create slaves from ENI/EXI, see esConfigureNetwork */
    EC_T_DWORD      dwCnfDataLen;                               /**< optional: create slaves from ENI/EXI, see esConfigureNetwork */

    EC_T_SIMULATOR_DEVICE_CONNECTION_DESC oDeviceConnection;    /**< See EC_SIMULATOR_DEVICE_CONNECTION_TYPE_... */
    EC_T_BOOL   bConnectHcGroups;                               /**< Connect hot connect groups in topology (floating group heads to free ports) */

    /* EC-Simulator core parameters */
    EC_T_DWORD  dwSimulatorAddress;                             /**< Reserved */
    EC_T_DWORD  dwBusCycleTimeUsec;                             /**< Cycle time of simulator job task */
    EC_T_BOOL   bDisableProcessDataImage;                       /**< Don't allocate Process Data Image at simulator (legacy support, CiA402 simulation) */
    EC_T_UINT64 qwOemKey;                                       /**< 64 bit OEM key (optional) */

    /* adapter parameters */
    EC_T_BYTE   abyMac[6];                                      /**< MAC address */
    EC_T_DWORD  dwRxBufferCnt;                                  /**< Frame buffer count for IST */

    /* application specific */
    EC_T_BOOL   bJobsExecutedByApp;                             /**< EC_FALSE: esExecJob explicitly called by application, EC_TRUE: implicitly by emllSimulator */

    /* license parameters */
    EC_T_CHAR   szLicenseKey[EC_SIMULATOR_KEY_SIZE];            /**< License key (zero terminated string) */
    EC_T_LINK_PARMS* apLinkParms[EC_SIMULATOR_MAX_LINK_PARMS];  /**< link parms of network adapters passed to EC-Simulator Core, e.g. for validation of MAC address of license key */

    /* AtesRasSrv parameters */
    EC_T_BOOL   bStartRasServer;
    EC_T_WORD   wRasServerPort;                                 /**< RAS server port */
    EC_T_CPUSET oRasCpuAffinityMask;                            /**< RAS server threads CPU affinity mask */
    EC_T_DWORD  dwRasPriority;                                  /**< RAS server threads priority */
    EC_T_DWORD  dwRasStackSize;                                 /**< RAS server threads stack size */

    /* Performance Measurement parameters*/
    EC_T_PERF_MEAS_INTERNAL_PARMS PerfMeasInternalParms;        /**< Internal performance measurement parameters */
} EC_PACKED_API EC_T_LINK_PARMS_SIMULATOR;
#include EC_PACKED_INCLUDESTOP

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_LINKIOCTL_SIMULATOR_ADD_S2S_COPY_DPRAM_IN
{
    EC_T_WORD    wSrcCfgFixedAddress;
    EC_T_DWORD   dwSrcDpramBitOffs;
    EC_T_WORD    wDstCfgFixedAddress;
    EC_T_DWORD   dwDstDpramBitOffs;
    EC_T_DWORD   dwBitSize;
} EC_PACKED_API EC_T_LINKIOCTL_SIMULATOR_ADD_S2S_COPY_DPRAM_IN;
#include EC_PACKED_INCLUDESTOP

#define EC_LINKIOCTL_SIMULATOR_ADD_S2S_COPY_DPRAM                      (0x00E00005)

/* Udp */
#define EC_LINK_PARMS_SIGNATURE_UDP_PATTERN (EC_T_DWORD)0x0000CB50
#define EC_LINK_PARMS_SIGNATURE_UDP_VERSION (EC_T_DWORD)0x00000001
#define EC_LINK_PARMS_SIGNATURE_UDP (EC_T_DWORD)(EC_LINK_PARMS_SIGNATURE|EC_LINK_PARMS_SIGNATURE_UDP_PATTERN|EC_LINK_PARMS_SIGNATURE_UDP_VERSION)
#define EC_LINK_PARMS_IDENT_UDP "Udp"

#define MAX_LEN_UDP_ADAPTER_NAME 64 /* deprecated */
#define EC_UDP_ADAPTER_NAME_MAXLEN 63
#define EC_UDP_ADAPTER_NAME_SIZE (EC_UDP_ADAPTER_NAME_MAXLEN + 1)

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_LINK_PARMS_UDP
{
    EC_T_LINK_PARMS linkParms;                                  /**< Common link parameters. Signature must be set to EC_LINK_PARMS_SIGNATURE_UDP */

    EC_T_CHAR       szAdapterName[EC_UDP_ADAPTER_NAME_SIZE];    /**< Adapter name (zero terminated) */

    EC_T_BYTE       abyIpAddress[4];                            /**< Adapter IP address */
    EC_T_WORD       wPort;                                      /**< Port number */
} EC_PACKED_API EC_T_LINK_PARMS_UDP;
#include EC_PACKED_INCLUDESTOP

/* Ndis */
#define EC_LINK_PARMS_SIGNATURE_NDIS_PATTERN (EC_T_DWORD)0x0000CB60
#define EC_LINK_PARMS_SIGNATURE_NDIS_VERSION (EC_T_DWORD)0x00000002
#define EC_LINK_PARMS_SIGNATURE_NDIS (EC_T_DWORD)(EC_LINK_PARMS_SIGNATURE|EC_LINK_PARMS_SIGNATURE_NDIS_PATTERN|EC_LINK_PARMS_SIGNATURE_NDIS_VERSION)
#define EC_LINK_PARMS_IDENT_NDIS "Ndis"

#define MAX_LEN_NDIS_ADAPTER_NAME 64 /* deprecated */
#define EC_NDIS_ADAPTER_NAME_MAXLEN 63
#define EC_NDIS_ADAPTER_NAME_SIZE (EC_NDIS_ADAPTER_NAME_MAXLEN + 1)

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_LINK_PARMS_NDIS
{
    EC_T_LINK_PARMS linkParms;                                      /**< Common link parameters. Signature must be set to EC_LINK_PARMS_SIGNATURE_NDIS */

    /* either abyIpAddress or szAdapterName or first adapter found is used:
       if abyIpAddress is set then abyIpAddress is used
       if abyIpAddress is not set, but szAdapterName is set then szAdapterName is used
       if abyIpAddress is not set and szAdapterName is not set then first adapter found is used */

    /* native adapter id, format: "{xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx}"
       see HKLM\SOFTWARE\Microsoft\Windows NT\CurrentVersion\NetworkCards\<n>:ServiceName
       .NET: System.Net.NetworkInformation.NetworkInterface.Id */
    EC_T_CHAR           szAdapterName[EC_NDIS_ADAPTER_NAME_SIZE];   /**< ServiceName of network adapter, see HKLM\SOFTWARE\Microsoft\Windows NT\CurrentVersion\NetworkCards in registry (zero terminated) */

    EC_T_BYTE           abyIpAddress[4];                            /**< IP address of network adapter */

    /* New parameters in version 2 */
    EC_T_BOOL           bDisablePromiscuousMode;                    /**< Disable adapter promiscuous mode */

    EC_T_BOOL           bDisableForceBroadcast;                     /**< Don't change target MAC address to FF:FF:FF:FF:FF:FF */
} EC_PACKED_API EC_T_LINK_PARMS_NDIS;
#include EC_PACKED_INCLUDESTOP

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_LINK_NDIS_VERSION_INFO
{
    EC_T_DWORD           dwNdisVersion;                             /**< Ndis version: The higher two bytes represent the Major Version and the lower two bytes the Minor Version */
    EC_T_DWORD           dwEcatNdisVersion;                         /**< Ecat Ndis version: For example V3.1.2.10 will be returned in the following form in Hex. 0x0301020A */
} EC_PACKED_API EC_T_LINK_NDIS_VERSION_INFO;
#include EC_PACKED_INCLUDESTOP

/*****************************************************************************/
/* Stm32Eth                                                                  */
/*****************************************************************************/
#define EC_LINK_PARMS_SIGNATURE_STM32ETH_PATTERN (EC_T_DWORD)0x0000CB70
#define EC_LINK_PARMS_SIGNATURE_STM32ETH_VERSION (EC_T_DWORD)0x00000001
#define EC_LINK_PARMS_SIGNATURE_STM32ETH (EC_T_DWORD)(EC_LINK_PARMS_SIGNATURE|EC_LINK_PARMS_SIGNATURE_STM32ETH_PATTERN|EC_LINK_PARMS_SIGNATURE_STM32ETH_VERSION)
#define EC_LINK_PARMS_IDENT_STM32ETH   "Stm32Eth"

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_LINK_PARMS_STM32ETH
{
    EC_T_LINK_PARMS     linkParms;          /**< Common link parameters. Signature must be set to EC_LINK_PARMS_SIGNATURE_STM32ETH */

    EC_T_DWORD          dwRxBuffersCnt;     /**< Receive buffer count */
    EC_T_DWORD          dwRxBufferLen;      /**< Recevie buffer size for a single Ethernet frame. */
    EC_T_DWORD          dwTxBuffersCnt;     /**< Transmit buffer count */
} EC_PACKED_API EC_T_LINK_PARMS_STM32ETH;
#include EC_PACKED_INCLUDESTOP

/* Microchip LAN743x      */
/* ==================== */
#define EC_LINK_PARMS_SIGNATURE_LAN743X_PATTERN (EC_T_DWORD)0x0000CB80
#define EC_LINK_PARMS_SIGNATURE_LAN743X_VERSION (EC_T_DWORD)0x00000001
#define EC_LINK_PARMS_SIGNATURE_LAN743X (EC_T_DWORD)(EC_LINK_PARMS_SIGNATURE|EC_LINK_PARMS_SIGNATURE_LAN743X_PATTERN|EC_LINK_PARMS_SIGNATURE_LAN743X_VERSION)
#define EC_LINK_PARMS_IDENT_LAN743X "LAN743x"

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_LINK_PARMS_LAN743X
{
    EC_T_LINK_PARMS linkParms;              /**< Common link parameters. Signature must be set to EC_LINK_PARMS_SIGNATURE_LAN743X */

    EC_T_BOOL       bNotUseDmaBuffers;      /**< EC_TRUE: copy buffer before processing, EC_FALSE: Use buffers from DMA (default) */

    EC_T_DWORD      dwRxBuffers;            /**< Receive buffer count. Must be a power of 2, maximum 1024 */
    EC_T_DWORD      dwTxBuffers;            /**< Transmit buffer count. Must be a power of 2, maximum 1024 */

} EC_PACKED_API EC_T_LINK_PARMS_LAN743X;
#include EC_PACKED_INCLUDESTOP

/* Broadcom BCMGENET      */
/* ==================== */
#define EC_LINK_PARMS_SIGNATURE_BCMGENET_PATTERN (EC_T_DWORD)0x0000CBA0
#define EC_LINK_PARMS_SIGNATURE_BCMGENET_VERSION (EC_T_DWORD)0x00000001
#define EC_LINK_PARMS_SIGNATURE_BCMGENET (EC_T_DWORD)(EC_LINK_PARMS_SIGNATURE|EC_LINK_PARMS_SIGNATURE_BCMGENET_PATTERN|EC_LINK_PARMS_SIGNATURE_BCMGENET_VERSION)
#define EC_LINK_PARMS_IDENT_BCMGENET "BcmGenet"

#include EC_PACKED_API_INCLUDESTART
typedef enum _EC_T_BCMGENET_TYPE
{
    eBCMGENET_BCM2711 = 0 /**< MAC on BCM2711 SoC */
} EC_T_BCMGENET_TYPE;
#include EC_PACKED_INCLUDESTOP

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_LINK_PARMS_BCMGENET
{
    EC_T_LINK_PARMS linkParms;              /**< Common link parameters. Signature must be set to EC_LINK_PARMS_SIGNATURE_BCMGENET */

} EC_PACKED_API EC_T_LINK_PARMS_BCMGENET;
#include EC_PACKED_INCLUDESTOP

/* Ti EnetLLD ICSSG     */
/* ==================== */
#define EC_LINK_PARMS_SIGNATURE_TIENETICSSG_PATTERN (EC_T_DWORD)0x0000CBB0
#define EC_LINK_PARMS_SIGNATURE_TIENETICSSG_VERSION (EC_T_DWORD)0x00000001
#define EC_LINK_PARMS_SIGNATURE_TIENETICSSG (EC_T_DWORD)(EC_LINK_PARMS_SIGNATURE|EC_LINK_PARMS_SIGNATURE_TIENETICSSG_PATTERN|EC_LINK_PARMS_SIGNATURE_TIENETICSSG_VERSION)
#define EC_LINK_PARMS_IDENT_TIENETICSSG "TiEnetIcssg"


#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_LINK_PARMS_TIENETICSSG
{
    EC_T_LINK_PARMS linkParms;              /**< Common link parameters. Signature must be set to EC_LINK_PARMS_SIGNATURE_TIENETICSSG */
    EC_T_BOOL       bMaster;                /**< EC_TRUE: Initialize MAC */

} EC_PACKED_API EC_T_LINK_PARMS_TIENETICSSG;
#include EC_PACKED_INCLUDESTOP

/*-FUNCTIONS DECLARATION-----------------------------------------------------*/
ATEMLL_API EC_T_DWORD emllRegisterAntaios(EC_T_LINK_DRV_DESC* pLinkDrvDesc, EC_T_DWORD dwLinkDrvDescSize);
ATEMLL_API EC_T_DWORD emllRegisterBcmGenet(EC_T_LINK_DRV_DESC* pLinkDrvDesc, EC_T_DWORD dwLinkDrvDescSize);
ATEMLL_API EC_T_DWORD emllRegisterCCAT(EC_T_LINK_DRV_DESC* pLinkDrvDesc, EC_T_DWORD dwLinkDrvDescSize);
ATEMLL_API EC_T_DWORD emllRegisterCPSW(EC_T_LINK_DRV_DESC* pLinkDrvDesc, EC_T_DWORD dwLinkDrvDescSize);
ATEMLL_API EC_T_DWORD emllRegisterDW3504(EC_T_LINK_DRV_DESC* pLinkDrvDesc, EC_T_DWORD dwLinkDrvDescSize);
ATEMLL_API EC_T_DWORD emllRegisterETSEC(EC_T_LINK_DRV_DESC* pLinkDrvDesc, EC_T_DWORD dwLinkDrvDescSize);
ATEMLL_API EC_T_DWORD emllRegisterFslFec(EC_T_LINK_DRV_DESC* pLinkDrvDesc, EC_T_DWORD dwLinkDrvDescSize);
ATEMLL_API EC_T_DWORD emllRegisterGEM(EC_T_LINK_DRV_DESC* pLinkDrvDesc, EC_T_DWORD dwLinkDrvDescSize);
ATEMLL_API EC_T_DWORD emllRegisterI8254x(EC_T_LINK_DRV_DESC* pLinkDrvDesc, EC_T_DWORD dwLinkDrvDescSize);
ATEMLL_API EC_T_DWORD emllRegisterI8255x(EC_T_LINK_DRV_DESC* pLinkDrvDesc, EC_T_DWORD dwLinkDrvDescSize);
ATEMLL_API EC_T_DWORD emllRegisterICSS(EC_T_LINK_DRV_DESC* pLinkDrvDesc, EC_T_DWORD dwLinkDrvDescSize);
ATEMLL_API EC_T_DWORD emllRegisterICSSG(EC_T_LINK_DRV_DESC* pLinkDrvDesc, EC_T_DWORD dwLinkDrvDescSize);
ATEMLL_API EC_T_DWORD emllRegisterNdisUio(EC_T_LINK_DRV_DESC* pLinkDrvDesc, EC_T_DWORD dwLinkDrvDescSize);
ATEMLL_API EC_T_DWORD emllRegisterProxy(EC_T_LINK_DRV_DESC* pLinkDrvDesc, EC_T_DWORD dwLinkDrvDescSize);
ATEMLL_API EC_T_DWORD emllRegisterR6040(EC_T_LINK_DRV_DESC* pLinkDrvDesc, EC_T_DWORD dwLinkDrvDescSize);
ATEMLL_API EC_T_DWORD emllRegisterRTL8139(EC_T_LINK_DRV_DESC* pLinkDrvDesc, EC_T_DWORD dwLinkDrvDescSize);
ATEMLL_API EC_T_DWORD emllRegisterRTL8169(EC_T_LINK_DRV_DESC* pLinkDrvDesc, EC_T_DWORD dwLinkDrvDescSize);
ATEMLL_API EC_T_DWORD emllRegisterSimulator(EC_T_LINK_DRV_DESC* pLinkDrvDesc, EC_T_DWORD dwLinkDrvDescSize);
ATEMLL_API EC_T_DWORD emllRegisterSockRaw(EC_T_LINK_DRV_DESC* pLinkDrvDesc, EC_T_DWORD dwLinkDrvDescSize);
ATEMLL_API EC_T_DWORD emllRegisterStm32Eth(EC_T_LINK_DRV_DESC* pLinkDrvDesc, EC_T_DWORD dwLinkDrvDescSize);
ATEMLL_API EC_T_DWORD emllRegisterTiEnetIcssg(EC_T_LINK_DRV_DESC* pLinkDrvDesc, EC_T_DWORD dwLinkDrvDescSize);
#endif /* INC_ECLINK */

/*-END OF SOURCE FILE--------------------------------------------------------*/
