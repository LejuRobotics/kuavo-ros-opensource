/*-----------------------------------------------------------------------------
 * EcEscReg.h               file
 * Copyright                acontis technologies GmbH, Ravensburg, Germany
 * Response                 Willig, Andreas
 * Description              ESC Controller Registers
 * Date                     2009/7/15::10:40
 *---------------------------------------------------------------------------*/

#ifndef INC_ECESCREG
#define INC_ECESCREG

/*-INCLUDES------------------------------------------------------------------*/

/*-DEFINES-------------------------------------------------------------------*/
#define ESC_PORT_A                                          ((EC_T_WORD)0)
#define ESC_PORT_B                                          ((EC_T_WORD)1)
#define ESC_PORT_C                                          ((EC_T_WORD)2)
#define ESC_PORT_D                                          ((EC_T_WORD)3)
#define ESC_PORT_COUNT                                      ((EC_T_WORD)4)
#define ESC_PORT_INVALID                                    ((EC_T_WORD)0xFF)

/*-TYPEDEFS------------------------------------------------------------------*/

/*-CLASS---------------------------------------------------------------------*/

/*-FUNCTION DECLARATION------------------------------------------------------*/

/*-LOCAL VARIABLES-----------------------------------------------------------*/

/*-REGISTERS-----------------------------------------------------------------*/

/* EtherCAT configuration register offsets */
#define ECREG_SC_TYPE                                       ((EC_T_WORD)0x0000)
#define ECREG_SC_REVISION                                   ((EC_T_WORD)0x0001)
#define ECREG_SC_BUILD                                      ((EC_T_WORD)0x0002)
#define ECREG_FMMUS_SUPPORTED                               ((EC_T_WORD)0x0004)
#define ECREG_SYNCM_SUPPORTED                               ((EC_T_WORD)0x0005)
#define ECREG_RAM_SIZE                                      ((EC_T_WORD)0x0006)
#define ECREG_PORT_DESCRIPTOR                               ((EC_T_WORD)0x0007)
#define ECREG_FEATURES_SUPPORTED                            ((EC_T_WORD)0x0008)
#define ECREG_STATION_ADDRESS                               ((EC_T_WORD)0x0010)
#define ECREG_STATION_ADDRESS_ALIAS                         ((EC_T_WORD)0x0012)
#define ECREG_RESET_ECAT                                    ((EC_T_WORD)0x0040)
#define ECREG_RESET_PDI                                     ((EC_T_WORD)0x0041)
#define ECREG_DL_CONTROL                                    ((EC_T_WORD)0x0100)
#define ECREG_DL_CONTROL1                                   ((EC_T_WORD)0x0101)
#define ECREG_DL_CONTROL2                                   ((EC_T_WORD)0x0102)
#define ECREG_DL_CONTROL3                                   ((EC_T_WORD)0x0103)
#define ECREG_DL_STATUS                                     ((EC_T_WORD)0x0110)
#define ECREG_AL_CONTROL                                    ((EC_T_WORD)0x0120)
#define ECREG_AL_STATUS                                     ((EC_T_WORD)0x0130)
#define ECREG_AL_STATUS_LO                                  ((EC_T_WORD)0x0130)
#define ECREG_AL_STATUSCODE                                 ((EC_T_WORD)0x0134)
#define ECREG_AL_STATUSCODE_LO                              ((EC_T_WORD)0x0134)
#define ECREG_PDI_CONTROL                                   ((EC_T_WORD)0x0140)

#define ECREG_SLV_ECATEVENTMASK                             ((EC_T_WORD)0x0200)
#define ECREG_SLV_ALEVENTMASK                               ((EC_T_WORD)0x0204)
#define ECREG_SLV_ECATEVENTREQUEST                          ((EC_T_WORD)0x0210)
#define ECREG_SLV_ALEVENTREQUEST                            ((EC_T_WORD)0x0220)

#define ECREG_SLV_RXERRCOUNTER                              ((EC_T_WORD)0x0300)
#define ECREG_SLV_FWDERRCOUNTER                             ((EC_T_WORD)0x0308)
#define ECREG_SLV_PROCESSINGUNITERRCOUNTER                  ((EC_T_WORD)0x030C)
#define ECREG_SLV_PDIERRCOUNTER                             ((EC_T_WORD)0x030D)
#define ECREG_SLV_LOSTLINKCOUNTER                           ((EC_T_WORD)0x0310)

#define ECREG_WATCHDOG_DIVIDER                              ((EC_T_WORD)0x0400)
#define ECREG_WATCHDOG_TIME_PDI                             ((EC_T_WORD)0x0410)
#define ECREG_WATCHDOG_TIME_PROCESS_DATA                    ((EC_T_WORD)0x0420)
#define ECREG_WATCHDOG_COUNTER_PROCESS_DATA                 ((EC_T_WORD)0x0442)
#define ECREG_WATCHDOG_COUNTER_PDI                          ((EC_T_WORD)0x0443)

#define ECM_SB_EEP_SLV_PDIACCSTATE                          ((EC_T_WORD)0x0500)
#define ECM_SB_EEP_SLV_CTRLSTATUS                           ((EC_T_WORD)0x0502)
#define ECM_SB_EEP_SLV_EEPADDRESS                           ((EC_T_WORD)0x0504)
#define ECM_SB_EEP_SLV_EEPDATA                              ((EC_T_WORD)0x0508)

/* FMMU configuration */
#define ECREG_FMMU_CONFIG                                   ((EC_T_WORD)0x0600)
#define ECREG_FMMU_CONFIG_SIZE                              ((EC_T_WORD)16)
#define ECREG_FMMU_MAX_NUMOF                                ((EC_T_WORD)16)

/* Sync Manager configuration */
#define ECREG_SYNCMANAGER_CONFIG                            ((EC_T_WORD)0x0800)
#define ECREG_SYNCMANAGER_MBX_OUT                           ((EC_T_WORD)0x0800)
#define ECREG_SYNCMANAGER_MBX_IN                            ((EC_T_WORD)0x0808)
#define ECREG_SYNCMANAGER0                                  ((EC_T_WORD)0x0800)
#define ECREG_SYNCMANAGER1                                  ((EC_T_WORD)0x0808)
#define ECREG_SYNCMANAGER2                                  ((EC_T_WORD)0x0810)
#define ECREG_SYNCMANAGER3                                  ((EC_T_WORD)0x0818)
#define ECREG_SYNCMANAGER4                                  ((EC_T_WORD)0x0820)
#define ECREG_SYNCMANAGER5                                  ((EC_T_WORD)0x0828)
#define ECREG_SYNCMANAGER6                                  ((EC_T_WORD)0x0830)
#define ECREG_SYNCMANAGER7                                  ((EC_T_WORD)0x0838)
#define ECREG_SYNCMANAGER8                                  ((EC_T_WORD)0x0840)
#define ECREG_SYNCMANAGER9                                  ((EC_T_WORD)0x0848)
#define ECREG_SYNCMANAGER10                                 ((EC_T_WORD)0x0850)
#define ECREG_SYNCMANAGER11                                 ((EC_T_WORD)0x0858)
#define ECREG_SYNCMANAGER12                                 ((EC_T_WORD)0x0860)
#define ECREG_SYNCMANAGER13                                 ((EC_T_WORD)0x0868)
#define ECREG_SYNCMANAGER14                                 ((EC_T_WORD)0x0870)
#define ECREG_SYNCMANAGER15                                 ((EC_T_WORD)0x0878)
#define ECREG_SYNCMANAGER_MAX_NUMOF                         ((EC_T_WORD)16)

/* DC */
#define ECM_DCS_REC_TIMEPORT0                               ((EC_T_WORD)0x0900)
#define ECM_DCS_REC_TIMEPORT1                               ((EC_T_WORD)0x0904)
#define ECM_DCS_REC_TIMEPORT2                               ((EC_T_WORD)0x0908)
#define ECM_DCS_REC_TIMEPORT3                               ((EC_T_WORD)0x090C)
#define ECM_DCS_SYSTEMTIME                                  ((EC_T_WORD)0x0910)
#define ECM_DCS_REC_TIMEPORTL0                              ((EC_T_WORD)0x0918)

#define ECM_DCS_SYSTIME_OFFSET                              ((EC_T_WORD)0x0920)
#define ECM_DCS_SYSTIME_DELAY                               ((EC_T_WORD)0x0928)

#define ECM_DCS_SPEEDCOUNT_START                            ((EC_T_WORD)0x0930)
#define ECM_DCS_SPEEDCOUNT_DIFF                             ((EC_T_WORD)0x0932)
#define ECM_DCS_SYSTIMEDIFF_FILTERDEPTH                     ((EC_T_WORD)0x0934)
#define ECM_DCS_SPEEDCOUNT_FILTERDEPTH                      ((EC_T_WORD)0x0935)
#define ECM_DCS_REC_LATCHMODE                               ((EC_T_WORD)0x0936)

/*#define ECM_DCS_DC_USER_P1                                ((EC_T_WORD)0x0981)*/

#define ECM_DCS_DC_UNIT_CONTROL                             ((EC_T_WORD)0x0980)
#define ECM_DCS_DC_ACTIVATION_REGISTER                      ((EC_T_WORD)0x0981)
#define ECM_DCS_DC_SYNCPULSEWIDTH                           ((EC_T_WORD)0x0982)

#define ECM_DCS_DC_WR_STARTCYCOP                            ((EC_T_WORD)0x0990)
#define ECM_DCS_DC_RD_TIMNEXTSYNC0                          ((EC_T_WORD)0x0990)

#define ECM_DCS_DC_CYCLETIME0                               ((EC_T_WORD)0x09A0)
#define ECM_DCS_DC_CYCLETIME1                               ((EC_T_WORD)0x09A4)

/*DC Latching*/
#define ECM_DCL_SYNCLATCH_CONFIGURATION                     ((EC_T_WORD)0x0151)
#define ECM_DCL_LATCHSTATUS_REGISTER                        ((EC_T_WORD)0x09AE)

#define ECREG_SYSTEMTIME                                    ((EC_T_WORD)0x0910)
#define ECREG_SYSTIME_DIFF                                  ((EC_T_WORD)0x092C)

/* DCL */                                                      #define ECREG_DCL_CTRL_LATCH                                ((EC_T_WORD)0x09A8)
#define ECREG_DCL_CTRL_LATCH1                               ((EC_T_WORD)0x09A9)

#define ECM_DCL_LTIME0_POSITIVE                             ((EC_T_WORD)0x09B0)
#define ECM_DCL_LTIME0_NEGATIVE                             ((EC_T_WORD)0x09B8)
#define ECM_DCL_LTIME1_POSITIVE                             ((EC_T_WORD)0x09C0)
#define ECM_DCL_LTIME1_NEGATIVE                             ((EC_T_WORD)0x09C8)

#define ECREG_DIGITAL_IO_OUTPUT_DATA                        ((EC_T_WORD)0x0F00)

/*-/REGISTERS----------------------------------------------------------------*/

/*-REGISTER BITDEFINITIONS---------------------------------------------------*/

/*****************************************************************************
 * 0x0000: SC_TYPE (ECREG_SC_TYPE)
 *****************************************************************************/
#define ESCTYPE_BKHF_ELOLD                                  ((EC_T_BYTE)0x01)
#define ESCTYPE_ESC10                                       ((EC_T_BYTE)0x02)
#define ESCTYPE_ESC20                                       ((EC_T_BYTE)0x02)
#define ESCTYPE_ESC10_20                                    ((EC_T_BYTE)0x02)
#define ESCTYPE_BKHF_EKOLD                                  ((EC_T_BYTE)0x03)
#define ESCTYPE_IPCORE                                      ((EC_T_BYTE)0x04)
#define ESCTYPE_IPCOREBKHF                                  ((EC_T_BYTE)0x05)
#define ESCTYPE_ET1100                                      ((EC_T_BYTE)0x11)
#define ESCTYPE_ET1200                                      ((EC_T_BYTE)0x12)
#define ESCTYPE_NETX100_500                                 ((EC_T_BYTE)0x80)
#define ESCTYPE_NETX50                                      ((EC_T_BYTE)0x81)
#define ESCTYPE_NETX5                                       ((EC_T_BYTE)0x82)
#define ESCTYPE_NETX51_52                                   ((EC_T_BYTE)0x83)
#define ESCTYPE_HILSCHER_RES1                               ((EC_T_BYTE)0x84)
#define ESCTYPE_NETX90                                      ((EC_T_BYTE)0x85)
#define ESCTYPE_HILSCHER_RES3                               ((EC_T_BYTE)0x86)
#define ESCTYPE_HILSCHER_RES4                               ((EC_T_BYTE)0x87)
#define ESCTYPE_HILSCHER_RES5                               ((EC_T_BYTE)0x88)
#define ESCTYPE_TI_SITARA                                   ((EC_T_BYTE)0x90)
#define ESCTYPE_TI                                          ESCTYPE_TI_SITARA
#define ESCTYPE_TI_C2000                                    ((EC_T_BYTE)0x91)
#define ESCTYPE_INFINEON                                    ((EC_T_BYTE)0x98)
#define ESCTYPE_RENESAS                                     ((EC_T_BYTE)0xA0)
#define ESCTYPE_INNOVASIC                                   ((EC_T_BYTE)0xA8)
#define ESCTYPE_HMS                                         ((EC_T_BYTE)0xB0)
#define ESCTYPE_PROFICHIP                                   ((EC_T_BYTE)0xB8)
#define ESCTYPE_MICROCHIP                                   ((EC_T_BYTE)0xC0)
#define ESCTYPE_ASIX                                        ((EC_T_BYTE)0xC8)
#define ESCTYPE_TRINAMIC                                    ((EC_T_BYTE)0xD0)

/*****************************************************************************
 * 0x0007: Port descriptor (ECREG_PORT_DESCRIPTOR)
 *****************************************************************************/
#define ECM_PORT_NOTIMPLEMENTED                             ((EC_T_BYTE)(0x00))
#define ECM_PORT_NOTCONFIGURED                              ((EC_T_BYTE)(0x01))
#define ECM_PORT_EBUS                                       ((EC_T_BYTE)(0x02))
#define ECM_PORT_MII                                        ((EC_T_BYTE)(0x03))
#define ECM_PORT_MASK                                       ((EC_T_BYTE)(0x03))

/*****************************************************************************
 * 0x0008: ESC Features Supported (ECREG_FEATURES_SUPPOR    TED)
 *****************************************************************************/
#define ECM_DLI_FMMUBITSUPPORT                              ((EC_T_BYTE)(0x01))
#define ECM_DLI_DCSUPPORT                                   ((EC_T_BYTE)(0x04))
#define ECM_DLI_DC64SUPPORT                                 ((EC_T_BYTE)(0x08))


/*****************************************************************************
 * 0x0100: DL CONTROL (ECREG_DL_CONTROL)
 *****************************************************************************/
#define ECM_DLCTRL_NONEC_FORWARDING                         ((EC_T_DWORD)0x00000001)
#define ECM_DLCTRL_TEMPORARY_USE                            ((EC_T_DWORD)0x00000002)
#define ECM_DLCTRL_LOOP_PORT0_SHIFT                         ((EC_T_DWORD)0x08)
#define ECM_DLCTRL_LOOP_PORT1_SHIFT                         ((EC_T_DWORD)0x0A)
#define ECM_DLCTRL_LOOP_PORT2_SHIFT                         ((EC_T_DWORD)0x0C)
#define ECM_DLCTRL_LOOP_PORT3_SHIFT                         ((EC_T_DWORD)0x0E)
#define ECM_DLCTRL_LOOP_PORTS_MASK                          ((EC_T_DWORD)0x00000003)

#define ECM_DLCTRL_LOOP_PORTX_SHIFT(x)                      ((EC_T_DWORD)(ECM_DLCTRL_LOOP_PORT0_SHIFT+(2*((x)%4))))
#define ECM_DLCTRL_LOOP_PORTX_MASK(x)                       ((EC_T_DWORD)(0x00000003<<ECM_DLCTRL_LOOP_PORTX_SHIFT((x))))

#define ECM_DLCTRL_RXFIFOSIZE_SHIFT                         ((EC_T_DWORD)0x10)
#define ECM_DLCTRL_RXFIFOSIZE_MASK                          ((EC_T_DWORD)0x00000007)
#define ECM_DLCTRL_EBUS_LOWJITTER                           ((EC_T_DWORD)0x00100000)
#define ECM_DLCTRL_USEALIAS                                 ((EC_T_DWORD)0x01000000)

#define ECM_DLCTRL_LOOP_PORT_VAL_AUTO                       ((EC_T_DWORD)0x0)
#define ECM_DLCTRL_LOOP_PORT_VAL_AUTOCLOSE                  ((EC_T_DWORD)0x1)
#define ECM_DLCTRL_LOOP_PORT_VAL_ALWAYSOPEN                 ((EC_T_DWORD)0x2)
#define ECM_DLCTRL_LOOP_PORT_VAL_ALWAYSCLOSED               ((EC_T_DWORD)0x3)

/*****************************************************************************
 * 0x0110: DL STATUS (ECREG_DL_STATUS)
 *****************************************************************************/
#define ECM_DLS_PDI_OPERATIONAL                             ((EC_T_WORD)0x0001)
#define ECM_DLS_PDI_WATCHDOG_STATUS                         ((EC_T_WORD)0x0002)
#define ECM_DLS_PDI_ENH_LINK_DETECTION                      ((EC_T_WORD)0x0004)
#define ECM_DLS_PDI_PHYSICAL_LINK_ON_PORT0                  ((EC_T_WORD)0x0010)
#define ECM_DLS_PDI_PHYSICAL_LINK_ON_PORT(x)                ((EC_T_WORD)(1<<(4+((x)%4))))

#define ECM_DLS_PDI_LOOP_PORT0                              ((EC_T_WORD)0x0100)
#define ECM_DLS_PDI_COM_ON_PORT0                            ((EC_T_WORD)0x0200)

#define ECM_DLS_PDI_LOOP_PORT(x)                            ((EC_T_WORD)(1<<(8+(2*((x)%4)))))
#define ECM_DLS_PDI_COM_ON_PORT(x)                          ((EC_T_WORD)(1<<(9+(2*((x)%4)))))

/*****************************************************************************
 * 0x0120: AL CONTROL (ECREG_AL_CONTROL)
 *****************************************************************************/
#define ECR_ALCTRL_ACK_ERROR_IND                            ((EC_T_BYTE)(0x10))
#define ECR_ALCTRL_DEVICE_ID_REQUEST                        ((EC_T_BYTE)(0x20))

/*****************************************************************************
 * 0x0130: AL STATUS (ECREG_AL_STATUS, ECREG_AL_STATUS_LO, ECREG_AL_STATUS_HI)
 *****************************************************************************/
 /** \defgroup DEVICE_STATES Slave device state's
 @{
 */
#define DEVICE_STATE_UNKNOWN                                ((EC_T_WORD)(0xFFFF&(~DEVICE_STATE_ERROR))) /**< Slave in unkown state */
#define DEVICE_STATE_INIT                                   ((EC_T_WORD)0x0001)                         /**< Slave in INIT state */
#define DEVICE_STATE_PREOP                                  ((EC_T_WORD)0x0002)                         /**< Slave in PREOP state */
#define DEVICE_STATE_BOOTSTRAP                              ((EC_T_WORD)0x0003)                         /**< Slave in BOOTSTRAP state */
#define DEVICE_STATE_SAFEOP                                 ((EC_T_WORD)0x0004)                         /**< Slave in SAFEOP state */
#define DEVICE_STATE_OP                                     ((EC_T_WORD)0x0008)                         /**< Slave in OP state */
#define DEVICE_STATE_ERROR                                  ((EC_T_WORD)0x0010)                         /**< Slave in error state */
/**@}*/
#define DEVICE_STATE_IDREQUEST                              ((EC_T_WORD)0x0020)
#define DEVICE_STATE_MASK                                   ((EC_T_WORD)0x000F)

#define SlaveDevStateText(nState) \
    ((nState)==DEVICE_STATE_UNKNOWN?"UNKNOWN": \
     ((nState&DEVICE_STATE_MASK)==DEVICE_STATE_INIT?"INIT": \
      ((nState&DEVICE_STATE_MASK)==DEVICE_STATE_PREOP?"PREOP": \
       ((nState&DEVICE_STATE_MASK)==DEVICE_STATE_BOOTSTRAP?"BOOTSTRAP": \
        ((nState&DEVICE_STATE_MASK)==DEVICE_STATE_SAFEOP?"SAFEOP": \
         ((nState&DEVICE_STATE_MASK)==DEVICE_STATE_OP?"OP": \
          ((nState&DEVICE_STATE_ERROR)==DEVICE_STATE_ERROR?"ERROR": \
           "INVALID" \
    )))))))

/*****************************************************************************
 * 0x0134: AL STATUS CODE (ECREG_AL_STATUSCODE, ECREG_AL_STATUSCODE_LO, ECREG_AL_STATUSCODE_HI)
 * See  ETG.1020, V1.2.0, Table 1: Description of AL Status Codes usage
 * See also EC_TXT_STATUSCODE_..., EcStatusCodeToString, ALSTATUSCODE_ in SSC
 *****************************************************************************/
#define DEVICE_STATUSCODE_NOERROR                           ((EC_T_WORD)0x0000)
#define DEVICE_STATUSCODE_ERROR                             ((EC_T_WORD)0x0001)
#define DEVICE_STATUSCODE_NO_MEMORY                         ((EC_T_WORD)0x0002)
#define DEVICE_STATUSCODE_INVALID_DEVICE_SETUP              ((EC_T_WORD)0x0003)
#define DEVICE_STATUSCODE_SII_EEPROM_INFORMATION_MISMATCH   ((EC_T_WORD)0x0006)
#define DEVICE_STATUSCODE_FIRMWARE_UPDATE_ERROR             ((EC_T_WORD)0x0007)
#define DEVICE_STATUSCODE_LICENSE_ERROR                     ((EC_T_WORD)0x000E)
#define DEVICE_STATUSCODE_INVREQSTATECNG                    ((EC_T_WORD)0x0011)
#define DEVICE_STATUSCODE_UNKREQSTATE                       ((EC_T_WORD)0x0012)
#define DEVICE_STATUSCODE_BOOTSTRAPNSUPP                    ((EC_T_WORD)0x0013)
#define DEVICE_STATUSCODE_NOVALIDFW                         ((EC_T_WORD)0x0014)
#define DEVICE_STATUSCODE_INVALIDMBXCNF1                    ((EC_T_WORD)0x0015)
#define DEVICE_STATUSCODE_INVALIDMBXCNF2                    ((EC_T_WORD)0x0016)
#define DEVICE_STATUSCODE_INVALIDSMCNF                      ((EC_T_WORD)0x0017)
#define DEVICE_STATUSCODE_NOVALIDIN                         ((EC_T_WORD)0x0018)
#define DEVICE_STATUSCODE_NOVALIDOUT                        ((EC_T_WORD)0x0019)
#define DEVICE_STATUSCODE_SYNCERROR                         ((EC_T_WORD)0x001A)
#define DEVICE_STATUSCODE_SMWATCHDOG                        ((EC_T_WORD)0x001B)
#define DEVICE_STATUSCODE_INVSMTYPES                        ((EC_T_WORD)0x001C)
#define DEVICE_STATUSCODE_INVOUTCONFIG                      ((EC_T_WORD)0x001D)
#define DEVICE_STATUSCODE_INVINCONFIG                       ((EC_T_WORD)0x001E)
#define DEVICE_STATUSCODE_INVWDCONFIG                       ((EC_T_WORD)0x001F)
#define DEVICE_STATUSCODE_SLVNEEDCOLDRS                     ((EC_T_WORD)0x0020)
#define DEVICE_STATUSCODE_SLVNEEDINIT                       ((EC_T_WORD)0x0021)
#define DEVICE_STATUSCODE_SLVNEEDPREOP                      ((EC_T_WORD)0x0022)
#define DEVICE_STATUSCODE_SLVNEEDSAFEOP                     ((EC_T_WORD)0x0023)
#define DEVICE_STATUSCODE_INVALID_INPUT_MAPPING             ((EC_T_WORD)0x0024)
#define DEVICE_STATUSCODE_INVALID_OUTPUT_MAPPING            ((EC_T_WORD)0x0025)
#define DEVICE_STATUSCODE_INCONSISTENT_SETTINGS             ((EC_T_WORD)0x0026)
#define DEVICE_STATUSCODE_FREERUN_NOT_SUPPORTED             ((EC_T_WORD)0x0027)
#define DEVICE_STATUSCODE_SYNCMODE_NOT_SUPPORTED            ((EC_T_WORD)0x0028)
#define DEVICE_STATUSCODE_FREERUN_NEEDS_THREEBUFFER_MODE    ((EC_T_WORD)0x0029)
#define DEVICE_STATUSCODE_BACKGROUND_WATCHDOG               ((EC_T_WORD)0x002A)
#define DEVICE_STATUSCODE_NO_VALID_INPUTS_AND_OUTPUTS       ((EC_T_WORD)0x002B)
#define DEVICE_STATUSCODE_FATAL_SYNC_ERROR                  ((EC_T_WORD)0x002C)
#define DEVICE_STATUSCODE_NO_SYNC_ERROR                     ((EC_T_WORD)0x002D)
#define DEVICE_STATUSCODE_CYCLE_TIME_TOO_SMALL              ((EC_T_WORD)0x002E)
                                                           #define DEVICE_STATUSCODE_INVDCSYNCCNFG                     ((EC_T_WORD)0x0030)
#define DEVICE_STATUSCODE_INVDCLATCHCNFG                    ((EC_T_WORD)0x0031)
#define DEVICE_STATUSCODE_PLLERROR                          ((EC_T_WORD)0x0032)
#define DEVICE_STATUSCODE_INVDCIOERROR                      ((EC_T_WORD)0x0033)
#define DEVICE_STATUSCODE_INVDCTOERROR                      ((EC_T_WORD)0x0034)
#define DEVICE_STATUSCODE_DC_INVALID_SYNC_CYCLE_TIME        ((EC_T_WORD)0x0035)
#define DEVICE_STATUSCODE_DC_SYNC0_CYCLE_TIME               ((EC_T_WORD)0x0036)
#define DEVICE_STATUSCODE_DC_SYNC1_CYCLE_TIME               ((EC_T_WORD)0x0037)

#define DEVICE_STATUSCODE_MBX_AOE                           ((EC_T_WORD)0x0041)
#define DEVICE_STATUSCODE_MBX_EOE                           ((EC_T_WORD)0x0042)
#define DEVICE_STATUSCODE_MBX_COE                           ((EC_T_WORD)0x0043)
#define DEVICE_STATUSCODE_MBX_FOE                           ((EC_T_WORD)0x0044)
#define DEVICE_STATUSCODE_MBX_SOE                           ((EC_T_WORD)0x0045)

#define DEVICE_STATUSCODE_MBX_VOE                           ((EC_T_WORD)0x004F)

#define DEVICE_STATUSCODE_EEPROM_NO_ACCESS                  ((EC_T_WORD)0x0050)
#define DEVICE_STATUSCODE_EEPROM_ERROR                      ((EC_T_WORD)0x0051)
#define DEVICE_STATUSCODE_EXT_HARDWARE_NOT_READY            ((EC_T_WORD)0x0052)

#define DEVICE_STATUSCODE_SLAVE_RESTARTED_LOCALLY           ((EC_T_WORD)0x0060)
#define DEVICE_STATUSCODE_DEVICE_IDENTIFICATION_UPDATED     ((EC_T_WORD)0x0061)

#define DEVICE_STATUSCODE_MODULE_ID_LIST_NOT_MATCH          ((EC_T_WORD)0x0070)

#define DEVICE_STATUSCODE_APPLICATION_CONTROLLER_AVAILABLE  ((EC_T_WORD)0x00F0)

#define SlaveDevStatusCodeText(nCode)                   \
    ecatGetText(((EC_T_DWORD)(EC_ALSTATUSCODEBASE+(nCode))))

/*****************************************************************************
 * 0x0140: PDI Control (ECREG_PDI_CONTROL)
 *****************************************************************************/
#define ECM_PDICTRL_PDI_MASK                                ((EC_T_WORD)0x00ff)
#define ECM_PDICTRL_DEVICE_EMULATION                        ((EC_T_WORD)0x0100)

/*****************************************************************************
 * 0x0200/0x210: ECAT Event Request (ECREG_SLV_ECATEVENTMASK, ECREG_SLV_ECATEVENTREQUEST)
 *****************************************************************************/
#define ECM_ECATEVENT_LATCH                                 ((EC_T_WORD)0x0001)
#define ECM_ECATEVENT_DLSTATUS                              ((EC_T_WORD)0x0004)
#define ECM_ECATEVENT_ALSTATUS                              ((EC_T_WORD)0x0008)
#define ECM_ECATEVENT_SMCHANNEL0                            ((EC_T_WORD)0x0010)
#define ECM_ECATEVENT_SMCHANNEL1                            ((EC_T_WORD)0x0020)
#define ECM_ECATEVENT_SMCHANNEL2                            ((EC_T_WORD)0x0040)
#define ECM_ECATEVENT_SMCHANNEL3                            ((EC_T_WORD)0x0080)
#define ECM_ECATEVENT_SMCHANNEL4                            ((EC_T_WORD)0x0100)
#define ECM_ECATEVENT_SMCHANNEL5                            ((EC_T_WORD)0x0200)
#define ECM_ECATEVENT_SMCHANNEL6                            ((EC_T_WORD)0x0400)
#define ECM_ECATEVENT_SMCHANNEL7                            ((EC_T_WORD)0x0800)

/*****************************************************************************
 * 0x0500: SSI PDI ACC STATE (ECM_SB_EEP_SLV_PDIACCSTATE)
 *****************************************************************************/
#define ECM_SB_EEP_PDIACCSTATE_PDI                          ((EC_T_WORD)0x0001)
#define ECM_SB_EEP_PDIACCSTATE_OVERRIDE                     ((EC_T_WORD)0x0002)
#define ECM_SB_EEP_PDIACCSTATE_PDIACTIVE                    ((EC_T_WORD)0x0100)

/*****************************************************************************
 * 0x0502: SSI CONTROL STATUS (ECM_SB_EEP_SLV_CTRLSTATUS)
 *****************************************************************************/
#define ECM_SB_EEP_CTRLSTATUS_WRITE_ENABLE                  ((EC_T_WORD)0x0001)

#define ECM_SB_EEP_CTRLSTATUS_WRITE_READACCESS              ((EC_T_WORD)0x0100)
#define ECM_SB_EEP_CTRLSTATUS_READ_READINPROGRESS           ((EC_T_WORD)0x0100)
#define ECM_SB_EEP_CTRLSTATUS_WRITE_WRITEACCESS             ((EC_T_WORD)0x0200)
#define ECM_SB_EEP_CTRLSTATUS_READ_WRITEINPROGRESS          ((EC_T_WORD)0x0200)
#define ECM_SB_EEP_CTRLSTATUS_WRITE_RELOADACCESS            ((EC_T_WORD)0x0400)
#define ECM_SB_EEP_CTRLSTATUS_READ_RELOADINPROGRESS         ((EC_T_WORD)0x0400)

/* busy flag is masked alone */
#define ECM_SB_EEP_CTRLSTATUS_ERRORMASK                     ((EC_T_WORD)0x7800)
#define ECM_SB_EEP_CTRLSTATUS_ERR_CHKSUM                    ((EC_T_WORD)0x0800)
#define ECM_SB_EEP_CTRLSTATUS_LOADING_STATUS                ((EC_T_WORD)0x1000)
#define ECM_SB_EEP_CTRLSTATUS_ERR_ACK                       ((EC_T_WORD)0x2000)
#define ECM_SB_EEP_CTRLSTATUS_ERR_WRITE_ENA                 ((EC_T_WORD)0x4000)
#define ECM_SB_EEP_CTRLSTATUS_BUSY                          ((EC_T_WORD)0x8000)

/*****************************************************************************
* 0x0600: SYNC MANAGER (ECREG_SYNCMAN_START)
*****************************************************************************/
#define ECREG_FMMU_LOGICAL_START_ADDR                       ((EC_T_DWORD)0x00000000)
#define ECREG_FMMU_LENGTH                                   ((EC_T_WORD)0x0004)
#define ECREG_FMMU_LOGICAL_START_BIT                        ((EC_T_BYTE)0x06)
#define ECREG_FMMU_LOGICAL_STOP_BIT                         ((EC_T_BYTE)0x07)
#define ECREG_FMMU_PHYSICAL_START_ADDR                      ((EC_T_WORD)0x0008)
#define ECREG_FMMU_PHYSICAL_START_BIT                       ((EC_T_BYTE)0x0A)
#define ECREG_FMMU_TYPE                                     ((EC_T_BYTE)0x0B)
#define ECREG_FMMU_ACTIVATE                                 ((EC_T_BYTE)0x0C)
#define ECREG_FMMU_RESERVED                                 ((EC_T_WORD)0x000D)

/*****************************************************************************
 * 0x0800: SYNC MANAGER (ECREG_SYNCMAN_START)
 *****************************************************************************/
#define ECREG_SYNCMANAGER_START                             ((EC_T_WORD)0x0000)
#define ECREG_SYNCMANAGER_LENGTH                            ((EC_T_WORD)0x0002)
#define ECREG_SYNCMANAGER_CTRL                              ((EC_T_WORD)0x0004)
#define ECREG_SYNCMANAGER_OPMODE_MASK                       ((EC_T_BYTE)0x03)
#define ECREG_SYNCMANAGER_OPMODE_BUF_MASK                   ((EC_T_BYTE)0x03)
#define ECREG_SYNCMANAGER_OPMODE_3BUF                       ((EC_T_BYTE)0x00)
#define ECREG_SYNCMANAGER_OPMODE_3BUF_M                     ((EC_T_BYTE)0x01)
#define ECREG_SYNCMANAGER_OPMODE_1BUF                       ((EC_T_BYTE)0x02)
#define ECREG_SYNCMANAGER_OPMODE_1BUF_F                     ((EC_T_BYTE)0x03)
#define ECREG_SYNCMANAGER_ACCESS_MASK                       ((EC_T_BYTE)0x0C)
#define ECREG_SYNCMANAGER_ACCESS_READ                       ((EC_T_BYTE)0x00)
#define ECREG_SYNCMANAGER_ACCESS_WRITE                      ((EC_T_BYTE)0x04)
#define ECREG_SYNCMANAGER_FLB_ENABLE                        ((EC_T_BYTE)0x10)
#define ECREG_SYNCMANAGER_PDI_ENABLE                        ((EC_T_BYTE)0x20)
#define ECREG_SYNCMANAGER_WD_TRIG_ENABLE                    ((EC_T_BYTE)0x40)
#define ECREG_SYNCMANAGER_1_R_P                             ((EC_T_BYTE)0x22)
#define ECREG_SYNCMANAGER_1_W_P                             ((EC_T_BYTE)0x26)
#define ECREG_SYNCMANAGER_1_W_T                             ((EC_T_BYTE)0x46)
#define ECREG_SYNCMANAGER_1_W_P_F                           ((EC_T_BYTE)0x27)
#define ECREG_SYNCMANAGER_3_R_P                             ((EC_T_BYTE)0x20)
#define ECREG_SYNCMANAGER_3_W_P                             ((EC_T_BYTE)0x24)
#define ECREG_SYNCMANAGER_3_W_T                             ((EC_T_BYTE)0x44)
#define ECREG_SYNCMANAGER_3_W_P_T                           ((EC_T_BYTE)0x64)
#define ECREG_SYNCMANAGER_3_R                               ((EC_T_BYTE)0x00)
#define ECREG_SYNCMANAGER_3_W                               ((EC_T_BYTE)0x04)
#define ECREG_SYNCMANAGER_1_R                               ((EC_T_BYTE)0x02)
#define ECREG_SYNCMANAGER_1_W                               ((EC_T_BYTE)0x06)
#define ECREG_SYNCMANAGER_1_W_F                             ((EC_T_BYTE)0x07)
#define ECREG_SYNCMANAGER_STATUS                            ((EC_T_WORD)0x0005)
#define ECREG_SYNCMANAGER_IRQ_WRITE                         ((EC_T_BYTE)0x01)
#define ECREG_SYNCMANAGER_IRQ_READ                          ((EC_T_BYTE)0x02)
#define ECREG_SYNCMANAGER_WATCHDOG                          ((EC_T_BYTE)0x04)
#define ECREG_SYNCMANAGER_1BUF_STATUS                       ((EC_T_BYTE)0x08)
#define ECREG_SYNCMANAGER_3BUF_MASK                         ((EC_T_BYTE)0x30)
#define ECREG_SYNCMANAGER_3BUF_1                            ((EC_T_BYTE)0x00)
#define ECREG_SYNCMANAGER_3BUF_2                            ((EC_T_BYTE)0x10)
#define ECREG_SYNCMANAGER_3BUF_3                            ((EC_T_BYTE)0x20)
#define ECREG_SYNCMANAGER_ACTIVATE                          ((EC_T_WORD)0x0006)
#define ECREG_SYNCMANAGER_DISABLE                           ((EC_T_BYTE)0x00)
#define ECREG_SYNCMANAGER_ENABLE                            ((EC_T_BYTE)0x01)
#define ECREG_SYNCMANAGER_MBX_TOGGLE                        ((EC_T_BYTE)0x02)
#define ECREG_SYNCMANAGER_SM_LATCH_ECAT                     ((EC_T_BYTE)0x40)
#define ECREG_SYNCMANAGER_SM_LATCH_PDI                      ((EC_T_BYTE)0x80)
#define ECREG_SYNCMANAGER_UC_STATUS                         ((EC_T_WORD)0x0007)


/*****************************************************************************
 * 0x0980: DC CYCLIC UNIT CONTROL (ECM_DCS_DC_UNIT_CONTROL)
 *****************************************************************************/
#define ECM_DCS_CTL_SYNCOUT                                 ((EC_T_BYTE)0x01)
#define ECM_DCS_CTL_LATCHIN0                                ((EC_T_BYTE)0x10)
#define ECM_DCS_CTL_LATCHIN1                                ((EC_T_BYTE)0x20)


/*****************************************************************************
 * 0x0981: DC ACTIVATION (ECM_DCS_DC_ACTIVATION_REGISTER)
 *****************************************************************************/
#define ECM_DCS_ACT_SYNCOUTACTIVATION                       ((EC_T_BYTE)0x01)
#define ECM_DCS_ACT_SYNC0GENERATION                         ((EC_T_BYTE)0x02)
#define ECM_DCS_ACT_SYNC1GENERATION                         ((EC_T_BYTE)0x04)
#define ECM_DCS_ACT_AUTOACTIVATION                          ((EC_T_BYTE)0x08)
#define ECM_DCS_ACT_STARTTIME_EXTENSION                     ((EC_T_BYTE)0x10)
#define ECM_DCS_ACT_STARTTIME_PLAUSIBILITYCHECK             ((EC_T_BYTE)0x20)
#define ECM_DCS_ACT_NEARFUTURE_CONFIGURATION                ((EC_T_BYTE)0x40)
#define ECM_DCS_ACT_SYNCSIGNAL_DEBUGPULSE_VASILI            ((EC_T_BYTE)0x80)

/*****************************************************************************
 * 0x09a8: DC LATCHING (ECREG_DCL_CTRL_LATCH)
 *****************************************************************************/
#define ECM_DCL_CTL_L0POS_SINGLE                            ((EC_T_BYTE)0)
#define ECM_DCL_CTL_L0NEG_SINGLE                            ((EC_T_BYTE)1)
#define ECM_DCL_CTL_L1POS_SINGLE                            ((EC_T_BYTE)8)
#define ECM_DCL_CTL_L1NEG_SINGLE                            ((EC_T_BYTE)9)

/*-/REGISTER BITDEFINITIONS--------------------------------------------------*/

/* SII (WORD) Offsets */
#define ESC_SII_REG_PDICONTROL                              ((EC_T_WORD)0x0000)
#define ESC_SII_REG_PDICONFIG                               ((EC_T_WORD)0x0001)
#define ESC_SII_REG_SYNCIMPULSELENGTH                       ((EC_T_WORD)0x0002)
#define ESC_SII_REG_EXTENDEDPDICONFIG                       ((EC_T_WORD)0x0003)
#define ESC_SII_REG_ALIASADDRESS                            ((EC_T_WORD)0x0004)
#define ESC_SII_REG_CHECKSUM                                ((EC_T_WORD)0x0007)
#define ESC_SII_REG_VENDORID                                ((EC_T_WORD)0x0008)
#define ESC_SII_REG_PRODUCTCODE                             ((EC_T_WORD)0x000A)
#define ESC_SII_REG_REVISIONNUMBER                          ((EC_T_WORD)0x000C)
#define ESC_SII_REG_REVISIONNUMBER_LO                       ((EC_T_WORD)0x000C)
#define ESC_SII_REG_REVISIONNUMBER_HI                       ((EC_T_WORD)0x000D)
#define ESC_SII_REG_SERIALNUMBER                            ((EC_T_WORD)0x000E)
#define ESC_SII_REG_BOOT_RECV_MBX                           ((EC_T_WORD)0x0014)
#define ESC_SII_REG_BOOT_RECV_MBX_OFFSET                    ((EC_T_WORD)0x0014)
#define ESC_SII_REG_BOOT_RECV_MBX_SIZE                      ((EC_T_WORD)0x0015)
#define ESC_SII_REG_BOOT_SEND_MBX                           ((EC_T_WORD)0x0016)
#define ESC_SII_REG_BOOT_SEND_MBX_OFFSET                    ((EC_T_WORD)0x0016)
#define ESC_SII_REG_BOOT_SEND_MBX_SIZE                      ((EC_T_WORD)0x0017)
#define ESC_SII_REG_STD_RECV_MBX                            ((EC_T_WORD)0x0018)
#define ESC_SII_REG_STD_RECV_MBX_OFFSET                     ((EC_T_WORD)0x0018)
#define ESC_SII_REG_STD_RECV_MBX_SIZE                       ((EC_T_WORD)0x0019)
#define ESC_SII_REG_STD_SEND_MBX                            ((EC_T_WORD)0x001A)
#define ESC_SII_REG_STD_SEND_MBX_OFFSET                     ((EC_T_WORD)0x001A)
#define ESC_SII_REG_STD_SEND_MBX_SIZE                       ((EC_T_WORD)0x001B)
#define ESC_SII_REG_MBX_PROTOCOL                            ((EC_T_WORD)0x001C)
#define ESC_SII_REG_FIRSTCATEGORYHDR                        ((EC_T_WORD)0x0040)

#define ESC_SII_ESC_CONFIG_AREA_WORDLEN                     ((EC_T_WORD)8)

/*-SII category types--------------------------------------------------------*/
#define ESC_SII_CAT_HDRSIZE                                 ((EC_T_WORD)2)
#define ESC_SII_CAT_NOP                                     ((EC_T_WORD)0)
#define ESC_SII_CAT_FMMU                                    ((EC_T_WORD)40)
#define ESC_SII_CAT_SYNCM                                   ((EC_T_WORD)41)
#define ESC_SII_CAT_TXPDO                                   ((EC_T_WORD)50)
#define ESC_SII_CAT_RXPDO                                   ((EC_T_WORD)51)
#define ESC_SII_CAT_DC                                      ((EC_T_WORD)60)
#define ESC_SII_CAT_END                                     ((EC_T_WORD)0xFFFF)

/* SII category FMMU --------------------------------------------------------*/
#define ESC_SII_CAT_FMMU_OUTPUT                             ((EC_T_BYTE)0x01)
#define ESC_SII_CAT_FMMU_INPUT                              ((EC_T_BYTE)0x02)
#define ESC_SII_CAT_FMMU_MAILBOX                            ((EC_T_BYTE)0x03)

/* SII category SYNCM / (BYTE) offsets---------------------------------------*/
#define ESC_SII_CAT_SYNCM_ENTRYSIZE                         ((EC_T_BYTE)0x08)
#define ESC_SII_CAT_SYNCM_REG_STARTADDRESS                  ((EC_T_BYTE)0x00)
#define ESC_SII_CAT_SYNCM_REG_LENGTH                        ((EC_T_BYTE)0x02)
#define ESC_SII_CAT_SYNCM_REG_CONTROL                       ((EC_T_BYTE)0x04)
#define ESC_SII_CAT_SYNCM_REG_ENABLE                        ((EC_T_BYTE)0x06)
#define ESC_SII_CAT_SYNCM_REG_TYPE                          ((EC_T_BYTE)0x07)

#define ESC_SII_CAT_SYNCM_ENABLE_ENABLED                    ((EC_T_BYTE)0x01)
#define ESC_SII_CAT_SYNCM_ENABLE_FIXCONTENT                 ((EC_T_BYTE)0x02)
#define ESC_SII_CAT_SYNCM_ENABLE_VIRTUAL                    ((EC_T_BYTE)0x04)
#define ESC_SII_CAT_SYNCM_ENABLE_OP_ONLY                    ((EC_T_BYTE)0x08)

#define ESC_SII_CAT_SYNCM_TYPE_NOTUSED                      ((EC_T_BYTE)0x00)
#define ESC_SII_CAT_SYNCM_TYPE_MBXOUT                       ((EC_T_BYTE)0x01)
#define ESC_SII_CAT_SYNCM_TYPE_MBXIN                        ((EC_T_BYTE)0x02)
#define ESC_SII_CAT_SYNCM_TYPE_PDOUT                        ((EC_T_BYTE)0x03)
#define ESC_SII_CAT_SYNCM_TYPE_PDIN                         ((EC_T_BYTE)0x04)

/* SII category TX/RX PDO / (BYTE) offsets-----------------------------------*/
#define ESC_SII_CAT_PDOLIST_SIZE                            ((EC_T_BYTE)0x08)
#define ESC_SII_CAT_PDOLIST_INDEX                           ((EC_T_BYTE)0x00)
#define ESC_SII_CAT_PDOLIST_NUMOFENTRIES                    ((EC_T_BYTE)0x02)
#define ESC_SII_CAT_PDOLIST_SYNCM                           ((EC_T_BYTE)0x03)

#define ESC_SII_CAT_PDO_ENTRYSIZE                           ((EC_T_BYTE)0x08)
#define ESC_SII_CAT_PDO_INDEX                               ((EC_T_BYTE)0x00)
#define ESC_SII_CAT_PDO_SUBINDEX                            ((EC_T_BYTE)0x02)
#define ESC_SII_CAT_PDO_NAMEIDX                             ((EC_T_BYTE)0x03)
#define ESC_SII_CAT_PDO_TYPEIDX                             ((EC_T_BYTE)0x04)
#define ESC_SII_CAT_PDO_BITLEN                              ((EC_T_BYTE)0x05)

#define ESC_SII_CAT_DC_ENTRYSIZE                            ((EC_T_BYTE)0x18)
#define ESC_SII_CAT_DC_SYNC0_CYCLE_TIME                     ((EC_T_BYTE)0x00)
#define ESC_SII_CAT_DC_SYNC0_SHIFT_TIME                     ((EC_T_BYTE)0x04)
#define ESC_SII_CAT_DC_SYNC1_SHIFT_TIME                     ((EC_T_BYTE)0x08)
#define ESC_SII_CAT_DC_SYNC1_CYCLE_FACTOR                   ((EC_T_BYTE)0x0C)
#define ESC_SII_CAT_DC_REG_ACTIVATION                       ((EC_T_BYTE)0x0E)
#define ESC_SII_CAT_DC_SYNC0_CYCLE_FACTOR                   ((EC_T_BYTE)0x10)
#define ESC_SII_CAT_DC_NAMEIDX                              ((EC_T_BYTE)0x12)
#define ESC_SII_CAT_DC_DESCIDX                              ((EC_T_BYTE)0x13)

#endif /* INC_ECESCREG */
    /*-END OF SOURCE FILE--------------------------------------------------------*/
