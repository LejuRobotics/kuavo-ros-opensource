/** ---------------------------------------------------------------------------
 * \file
 * \brief       Generic control interface description
 * \author      Steven Krug
 * \copyright   acontis technologies GmbH, Weingarten, Germany
 *---------------------------------------------------------------------------*/

#ifndef INC_ECIOCTL
#define INC_ECIOCTL

#ifndef INC_ECTYPE
#include "EcType.h"
#endif

/* EtherCat specific control codes */
#define EC_IOCTL_GENERIC                                0x00000000
#define EC_IOCTL_DC                                     0x00030000
#define EC_IOCTL_SB                                     0x00050000
#define EC_IOCTL_HC                                     0x00060000
#define EC_IOCTL_DCM                                    0x00070000
#define EC_IOCTL_USER                                   0x00F00000  /* for user extension */
#define EC_IOCTL_PRIVATE                                0x00FF0000  /* private, internal IOCTL values */
#define EC_IOCTL_LINKLAYER                              0xCA000000
#define EC_IOCTL_LINKLAYER_MAIN                         EC_IOCTL_LINKLAYER
#define EC_IOCTL_LINKLAYER_RED                          0xCB000000
#define EC_IOCTL_LINKLAYER_IDX_MASK                     0x00FF0000
#define EC_IOCTL_LINKLAYER_IDX_SHIFT                            16
#define EC_IOCTL_LINKLAYER_CODE_MASK                    0x0000FFFF
#define EC_IOCTL_LINKLAYER_LAST                         0xCBFFFFFF
#define EC_IOCTL_SIMULATOR                              0xCC000000
#define EC_IOCTL_SIMULATOR_LAST                         0xCCFFFFFF
#define EC_IOCTL_MONITOR                                0xCD000000
#define EC_IOCTL_MONITOR_LAST                           0xCDFFFFFF
#define EC_IOCTL_PRIVATE2                               0xCE000000  /* private, internal IOCTL values */
/*      EC_IOCTL_REGISTERCLIENT                         (EC_IOCTL_GENERIC |  2) obsolete
        EC_IOCTL_UNREGISTERCLIENT                       (EC_IOCTL_GENERIC |  3) obsolete */

/********************************************************************************/
/** \brief Determine whether the main link or redundancy link is connected.
 * \note  See also EC_IOCTL_IS_MAIN_LINK_CONNECTED, EC_IOCTL_IS_RED_LINK_CONNECTED
 * \param pbyInBuf          [in]  Should be set to EC_NULL
 * \param dwInBufSize       [in]  Should be set to 0
 * \param pbyOutBuf         [out] Pointer to EC_T_DWORD or EC_T_LINK_CONNECTED_INFO. EC_T_DWORD: If value is EC_TRUE link is connected, if EC_FALSE it is not.
 * \param dwOutBufSize      [in]  Size of the output buffer in bytes (sizeof(EC_T_DWORD) / sizeof(EC_T_LINK_CONNECTED_INFO))
 * \param pdwNumOutData     [out] Pointer to EC_T_DWORD. Amount of bytes written to the output buffer.
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_ISLINK_CONNECTED                       (EC_IOCTL_GENERIC |  6)

/********************************************************************************/
/** \brief Enable or disable EC_NOTIFY_FRAME_RESPONSE_ERROR for specific errors. 
 * \note  #EC_FRAME_RESPONSE_ERROR_NOTIFY_MASK_DEFAULT: all errors except #EC_FRAME_RESPONSE_ERROR_NOTIFY_MASK_NON_ECAT_FRAME enabled.
 * \param pbyInBuf          [in]  Error enable bit mask (EC_T_DWORD)
 * \param dwInBufSize       [in]  Size of the input buffer provided at pbyInBuf in bytes
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_SET_FRAME_RESPONSE_ERROR_NOTIFY_MASK   (EC_IOCTL_GENERIC |  8)

#define EC_IOCTL_LINKLAYER_DBG_MSG                      (EC_IOCTL_GENERIC | 10) /* obsolete */

/********************************************************************************/
/** \brief Reset slave state machine
 * \param pbyInBuf          [in]  Pointer to an EC_T_DWORD type value containing the slave ID
 * \param dwInBufSize       [in]  Size of the input buffer provided at pbyInBuf in bytes
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_RESET_SLAVE                            (EC_IOCTL_GENERIC | 13)

#define EC_IOCTL_SLAVE_LINKMESSAGES                     (EC_IOCTL_GENERIC | 14) /* obsolete */

/********************************************************************************/
/** \brief Get cyclic configuration information from ENI file.
 * \param pbyInBuf          [in]  Pointer to dwCycEntryIndex: Cyclic entry index for which to get information
 * \param dwInBufSize       [in]  Size of the input buffer provided at pbyInBuf in bytes
 * \param pbyOutBuf         [out] Pointer to EC_T_CYC_CONFIG_DESC data type
 * \param dwOutBufSize      [in]  Size of the output buffer provided at pbyOutBuf in bytes
 * \param pdwNumOutData     [out] Pointer to EC_T_DWORD. Amount of bytes written to the output buffer.
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_GET_CYCLIC_CONFIG_INFO                 (EC_IOCTL_GENERIC | 15)

/********************************************************************************/
/** \brief Get Real-time Ethernet Driver mode (EcLinkMode_POLLING, EcLinkMode_INTERRUPT)
 * \param pbyInBuf          [in]  Should be set to EC_NULL
 * \param dwInBufSize       [in]  Should be set to 0
 * \param pbyOutBuf         [out] Pointer to struct EC_T_LINKLAYER_MODE_DESC
 * \param dwOutBufSize      [in]  Size of the output buffer in bytes
 * \param pdwNumOutData     [out] Pointer to EC_T_DWORD. Amount of bytes written to the output buffer.
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_GET_LINKLAYER_MODE                     (EC_IOCTL_GENERIC | 16)

/********************************************************************************/
/** \brief Determine if any slave to slave communication is configured.
 * \param pbyInBuf          [in]  Should be set to EC_NULL
 * \param dwInBufSize       [in]  Should be set to 0
 * \param pbyOutBuf         [out] Pointer to EC_T_DWORD. If value is EC_TRUE slave to slave communication is configured, if EC_FALSE it is not.
 * \param dwOutBufSize      [in]  Size of the output buffer in bytes
 * \param pdwNumOutData     [out] Pointer to EC_T_DWORD. Amount of bytes written to the output buffer.
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_IS_SLAVETOSLAVE_COMM_CONFIGURED        (EC_IOCTL_GENERIC | 17)

/********************************************************************************/
/** \brief Trigger read AL Status (ADO 0x0130) from all slaves
 * \param pbyInBuf[in]  Should be set to EC_NULL
 * \param dwInBufSize[in]  Should be set to 0
 * \param pbyOutBuf[out] Should be set to EC_NULL
 * \param dwOutBufSize[in]  Should be set to 0
 * \param pdwNumOutData[out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_INITIATE_UPDATE_ALL_SLAVE_STATE        (EC_IOCTL_GENERIC | 19)

/********************************************************************************/
/** \brief Add Sync Window Monitoring command to cyclic frame (BRD ADO 0x092C)
 * \param pbyInBuf[in]  Should be set to EC_NULL
 * \param dwInBufSize[in]  Should be set to 0
 * \param pbyOutBuf[out] Should be set to EC_NULL
 * \param dwOutBufSize[in]  Should be set to 0
 * \param pdwNumOutData[out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_ADD_BRD_SYNC_WINDOW_MONITORING         (EC_IOCTL_GENERIC | 20)

/* TODO: explain Only process data in input image */
#define EC_IOCTL_ONLY_PROCESS_DATA_IN_IMAGE             (EC_IOCTL_GENERIC | 21)

/********************************************************************************/
/** \brief Register callback for all cyclic frames received. Typically this is used when the Real-time Ethernet Driver operates in interrupt mode to get an event when the new input data (cyclic frame) is available.
 * The callback function has to be registered after the stack initialization and before starting the job task.
 * \param pbyInBuf          [in]  Cyclic frame received callback descriptor (EC_T_CYCFRAME_RX_CBDESC)
 * \param dwInBufSize       [in]  Size of the input buffer provided at pbyInBuf in bytes
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_REGISTER_CYCFRAME_RX_CB                (EC_IOCTL_GENERIC | 22)

/********************************************************************************/
/** \brief Determine whether the main link is connected.
 * \param pbyInBuf          [in]  Should be set to EC_NULL
 * \param dwInBufSize       [in]  Should be set to 0
 * \param pbyOutBuf         [out] Pointer to EC_T_DWORD. If value is EC_TRUE link is connected, if EC_FALSE it is not.
 * \param dwOutBufSize      [in]  Should be set to sizeof(EC_T_DWORD)
 * \param pdwNumOutData     [out] Pointer to EC_T_DWORD. Amount of bytes written to the output buffer.
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_IS_MAIN_LINK_CONNECTED                 (EC_IOCTL_GENERIC | 24)

/********************************************************************************/
/** \brief Determine whether the redundancy link is connected.
 * \param pbyInBuf          [in]  Should be set to EC_NULL
 * \param dwInBufSize       [in]  Should be set to 0
 * \param pbyOutBuf         [out] Pointer to EC_T_DWORD. If value is EC_TRUE link is connected, if EC_FALSE it is not.
 * \param dwOutBufSize      [in]  Should be set to sizeof(EC_T_DWORD)
 * \param pdwNumOutData     [out] Pointer to EC_T_DWORD. Amount of bytes written to the output buffer.
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_IS_RED_LINK_CONNECTED                  (EC_IOCTL_GENERIC | 25)

/********************************************************************************/
/** \brief Add CoE init commands dynamically.
 * \param pbyInBuf          [in]  Add CoE init command parameters (EC_T_ADD_COE_INITCMD_DESC)
 * \param dwInBufSize       [in]  Size of the input buffer provided at pbyInBuf in bytes
 * \param pbyInBuf          [in]  Should be set to EC_NULL
 * \param dwInBufSize       [in]  Should be set to 0
 * \param pbyOutBuf         [out] Pointer to EC_T_DWORD. If value is EC_TRUE link is connected, if EC_FALSE it is not.
 * \param dwOutBufSize      [in]  Should be set to sizeof(EC_T_DWORD)
 * \param pdwNumOutData     [out] Pointer to EC_T_DWORD. Amount of bytes written to the output buffer.
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_ADD_COE_INITCMD                        (EC_IOCTL_GENERIC | 26)

/********************************************************************************/
/** \brief Get the process data image size. This information may be used to provide process data image storage from outside the core. This IOCTL is to be called after network configuration.
 * \param pbyInBuf          [in]  Should be set to EC_NULL
 * \param dwInBufSize       [in]  Should be set to 0
 * \param pbyOutBuf         [out] Pointer to memory where the memory size information will be stored (type: EC_T_MEMREQ_DESC)
 * \param dwOutBufSize      [in]  Size of the output buffer in bytes
 * \param pdwNumOutData     [out] Pointer to EC_T_DWORD. Amount of bytes written to the output buffer.
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_GET_PDMEMORYSIZE                       (EC_IOCTL_GENERIC | 40)

/********************************************************************************/
/** \brief This function call registers an external memory provider to the stack, this memory will be used to store process data. If no memory provider is registered the stack will internally allocate the necessary amount of memory.
 * The function #EC_IOCTL_GET_PDMEMORYSIZE should be executed to determine the amount of memory the stack needs to store process data values. 
 * An external memory provider may additionally supply some hooks to give the stack a possibility to synchronize memory access with the application.
 * Also the memory provider has to be registered after configuring the network but prior to registering any client. Every client that registers with the stack will get back the memory pointers to PDOut/PDIn data registered within this call.
 * \param pbyInBuf          [in]  Memory provider (EC_T_MEMPROV_DESC)
 * \param dwInBufSize       [in]  Size of the input buffer provided at pbyInBuf in bytes
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_REGISTER_PDMEMORYPROVIDER              (EC_IOCTL_GENERIC | 41)

#define EC_IOCTL_FORCE_BROADCAST_DESTINATION            (EC_IOCTL_GENERIC | 42) /* obsolete */

/* Slave Statistics Retrieval */

/********************************************************************************/
/** \brief Set Slave Statistics read period [ms] (EC_T_DWORD). 0: disable.
 * \note Triggers an immediate read.
 * \param pbyInBuf          [in]  Slave Statistics read period [ms] (EC_T_DWORD)
 * \param dwInBufSize       [in]  Size of the input buffer provided at pbyInBuf in bytes
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_SET_SLVSTAT_PERIOD                     (EC_IOCTL_GENERIC | 43)

/********************************************************************************/
/** \brief Trigger slave statistics read.
 * \param pbyInBuf          [in]  Should be set to EC_NULL
 * \param dwInBufSize       [in]  Should be set to 0
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_FORCE_SLVSTAT_COLLECTION               (EC_IOCTL_GENERIC | 44)

/********************************************************************************/
/** \brief Get slave statistics showing errors on Ethernet layer for slave id. Statistics are read on a regular basis (default: off).
 * \note See also EC_IOCTL_SET_SLVSTAT_PERIOD, EC_IOCTL_FORCE_SLVSTAT_COLLECTION, EC_IOCTL_GET_SLVSTAT_PERIOD
 * \param pbyInBuf          [in]  Pointer to an EC_T_DWORD type variable containing the slave id
 * \param dwInBufSize       [in]  Size of the input buffer provided at pbyInBuf in bytes
 * \param pbyOutBuf         [out] Pointer to struct EC_T_SLVSTATISTICS_DESC
 * \param dwOutBufSize      [in]  Size of the output buffer provided at pbyOutBuf in bytes
 * \param pdwNumOutData     [out] Pointer to EC_T_DWORD. Amount of bytes written to the output buffer.
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_GET_SLVSTATISTICS                      (EC_IOCTL_GENERIC | 45)

/********************************************************************************/
/** \brief Clear slave statistics in slaves.
 * \param pbyInBuf          [in]  Should be set to EC_NULL
 * \param dwInBufSize       [in]  Should be set to 0
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_CLR_SLVSTATISTICS                      (EC_IOCTL_GENERIC | 46)

/********************************************************************************/
/** \brief Sets the mailbox retry count for a specific slave. If a slave rejects a mailbox access because of a busy state, the master retries that often.
 * \param pbyInBuf          [in]  Pointer to struct #EC_T_SET_MBX_RETRYACCESS_COUNT_DESC
 * \param pbyInBuf          [in]  Pointer to a size 6 byte array. The first 4 bytes must contain the slave id (EC_T_DWORD), the last 2 bytes the new retry count(EC_T_WORD).
 * \param dwInBufSize       [in]  Size of the input buffer provided at pbyInBuf in bytes
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_SET_MBX_RETRYACCESS_COUNT              (EC_IOCTL_GENERIC | 47)

/********************************************************************************/
/** \brief Sets the mailbox retry access period [ms] for a specific slave. If a slave rejects a mailbox access because of a busy state, the master restarts mailbox access after that period of time.
 * \param pbyInBuf          [in]  Pointer to a size 6 byte array. The first 4 bytes must contain the slave id (EC_T_DWORD), the last 2 bytes the new retry access period in milliseconds(EC_T_WORD).
 * \param dwInBufSize       [in]  Size of the input buffer provided at pbyInBuf in bytes
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_SET_MBX_RETRYACCESS_PERIOD             (EC_IOCTL_GENERIC | 48)

/********************************************************************************/
/** \brief Specifies if all the slaves must reach the requested master state.
 * \param pbyInBuf          [in]  Pointer to EC_T_BOOL variable. If set to EC_TRUE all slaves must reach the master requested state, if set to EC_FALSE the master can reach the requested state even if some slaves are missing or cannot reach the requested state.
 * \param dwInBufSize       [in]  Size of the input buffer provided at pbyInBuf in bytes
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_ALL_SLAVES_MUST_REACH_MASTER_STATE     (EC_IOCTL_GENERIC | 49)

#define EC_IOCTL_SET_NOTIFICATION_CTL                   (EC_IOCTL_GENERIC | 50) /* obsolete */

/********************************************************************************/
/** \brief Sets the value of the Object Entry with the given Index and SubIndex in the MainDevice Object Dictionary.
 * \note  Cannot be used by a RAS Client application, nor for Object Entries located at SubDevices. Complete access is not supported.
 * \param pbyInBuf          [in]  Pointer to struct #EC_T_MASTEROD_OBJECT_PARMS
 * \param dwInBufSize       [in]  Size of the input buffer provided at pbyInBuf in bytes
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_MASTEROD_SET_VALUE                     (EC_IOCTL_GENERIC | 51)

/********************************************************************************/
/** \brief Set the cyclic frames layout.
 * \param pbyInBuf          [in]  Pointer to an #EC_T_CYCFRAME_LAYOUT value containing the cyclic frame layout
 * \param dwInBufSize       [in]  Size of the input buffer provided at pbyInBuf in bytes
 * \param pbyOutBuf         [out] Pointer to EC_T_BOOL to carry out current enable set
 * \param dwOutBufSize      [in]  Size of the output buffer provided at pbyOutBuf in bytes
 * \param pdwNumOutData     [out] Pointer to EC_T_DWORD. Amount of bytes written to the output buffer.
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_SET_CYCFRAME_LAYOUT                    (EC_IOCTL_GENERIC | 52)

/********************************************************************************/
/** \brief Set notification enabled state. With #EC_T_SET_NOTIFICATION_ENABLED_PARMS::dwCode set to #EC_ALL_NOTIFICATIONS, all notifications can be changed at once. 
EC_T_SET_NOTIFICATION_ENABLED_PARMS::dwEnabled set to #EC_NOTIFICATION_DEFAULT, resets to default.
 * \param pbyInBuf          [in]  Pointer to EC_T_SET_NOTIFICATION_ENABLED_PARMS.
 * \param dwInBufSize       [in]  Size of the input buffer provided at pbyInBuf in bytes
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out]  Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_SET_NOTIFICATION_ENABLED               (EC_IOCTL_GENERIC | 53)

/********************************************************************************/
/** \brief The enabled state of notifications can be retrieved using #EC_IOCTL_GET_NOTIFICATION_ENABLED.
 * \param pbyInBuf          [in]  Pointer to #EC_T_GET_NOTIFICATION_ENABLED_PARMS
 * \param dwInBufSize       [in]  Size of the input buffer provided at pbyInBuf in bytes
 * \param pbyOutBuf         [out] Pointer to EC_T_BOOL to carry out current enable set
 * \param dwOutBufSize      [in]  Size of the output buffer provided at pbyOutBuf in bytes
 * \param pdwNumOutData     [out] Pointer to EC_T_DWORD. Amount of bytes written to the output buffer
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_GET_NOTIFICATION_ENABLED               (EC_IOCTL_GENERIC | 54)

/********************************************************************************/
/** \brief Set master default timeouts.
 * \param pbyInBuf          [in]  Pointer to #EC_T_MASTERDEFAULTTIMEOUTS_DESC
 * \param dwInBufSize       [in]  Size of the input buffer provided at pbyInBuf in bytes
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_SET_MASTER_DEFAULT_TIMEOUTS            (EC_IOCTL_GENERIC | 55)

/********************************************************************************/
/** \brief Set copy info processed in either #eUsrJob_SendAllCycFrames or in #eUsrJob_ProcessAllRxFrames.
 * \param pbyInBuf          [in]  Pointer to EC_T_BOOL. EC_TRUE: SendCycFrames, EC_FALSE: ProcessAllRxFrames.
 * \param dwInBufSize       [in]  Size of the input buffer provided at pbyInBuf in bytes
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_SET_COPYINFO_IN_SENDCYCFRAMES          (EC_IOCTL_GENERIC | 56)

/********************************************************************************/
/** \brief Set bus cycle time [us] master parameter without calling emInitMaster() again.
 * \param pbyInBuf          [in]  Pointer to value of EC_T_DWORD. Value may not be 0!
 * \param dwInBufSize       [in]  Size of the input buffer provided at pbyInBuf in bytes
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_SET_BUS_CYCLE_TIME                     (EC_IOCTL_GENERIC | 57)

/********************************************************************************/
/** \brief Enable or disable additional variables for specific data types. Default: Enabled.
 * \param pbyInBuf          [in]  Pointer to value of EC_T_BOOL. EC_TRUE: enable, EC_FALSE: disable.
 * \param dwInBufSize       [in]  Size of the input buffer provided at pbyInBuf in bytes
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_ADDITIONAL_VARIABLES_FOR_SPECIFIC_DATA_TYPES   (EC_IOCTL_GENERIC | 58)

/** \brief Set ignore INPUTs on WKC error
 * \param pbyInBuf          [in]  Pointer to value of EC_T_BOOL. EC_TRUE: Ignore INPUTs on WKC error.
 * \param dwInBufSize       [in]  Size of the input buffer provided at pbyInBuf in bytes
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 * */
#define EC_IOCTL_SET_IGNORE_INPUTS_ON_WKC_ERROR         (EC_IOCTL_GENERIC | 59)

/********************************************************************************/
/** \brief Enable or disable creation of "assign EEPROM back to ECAT" InitCmd if ENI generated based on bus-scan result.
 * \param pbyInBuf          [in]  Pointer to value of EC_T_BOOL. EC_TRUE: generate InitCmd.
 * \param dwInBufSize       [in]  Size of the input buffer provided at pbyInBuf in bytes
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_SET_GENENI_ASSIGN_EEPROM_BACK_TO_ECAT  (EC_IOCTL_GENERIC | 60)

/********************************************************************************/
/** \brief Specifies if slave errors must be automatically acknowledged.
 * \param pbyInBuf          [in]  Pointer to EC_T_BOOL variable. If set to EC_TRUE slave errors must be automatically acknowledged, if set to EC_FALSE the application must acknowledge slave errors explicitly.
 * \param dwInBufSize       [in]  Size of the input buffer provided at pbyInBuf in bytes
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_SET_AUTO_ACK_AL_STATUS_ERROR_ENABLED   (EC_IOCTL_GENERIC | 61)

/********************************************************************************/
/** \brief Specifies if the cyclic commands expected WKC must be automatically adjusted according to the state and the presence of the slaves.
 * \param pbyInBuf          [in]  Pointer to EC_T_BOOL variable. If set to EC_TRUE cyclic commands expected WKC must be automatically adjusted, if set to EC_FALSE the cyclic commands expected WKC stays unchanged.
 * \param dwInBufSize       [in]  Size of the input buffer provided at pbyInBuf in bytes
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_SET_AUTO_ADJUST_CYCCMD_WKC_ENABLED     (EC_IOCTL_GENERIC | 62)

/********************************************************************************/
/** \brief Reset Master Info Counters according to given bit masks.
 * \param pbyInBuf          [in]  Pointer to a value of EC_T_CLEAR_MASTER_INFO_COUNTERS_PARMS
 * \param dwInBufSize       [in]  Size of the input buffer provided at pbyInBuf in bytes
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_CLEAR_MASTER_INFO_COUNTERS             (EC_IOCTL_GENERIC | 63)

/********************************************************************************/
/** \brief Enable or disable the split frame processing. Default: Disabled.
If split frame processing is enabled the master allocates several buffers to store cyclic and acyclic EtherCAT frames. The size of these buffers depends on the number of cyclic frames in the ENI and the parameter EC_T_INIT_MASTER_PARMS::dwMaxAcycFramesQueued configured in emInitMaster(). 
The functionality should be enabled between emInitMaster() and the start of the job task.
 * \param pbyInBuf          [in]  Pointer to value of EC_T_BOOL. EC_TRUE: enable, EC_FALSE: disable.
 * \param dwInBufSize       [in]  Size of the input buffer provided at pbyInBuf in bytes
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_SET_SPLIT_FRAME_PROCESSING_ENABLED             (EC_IOCTL_GENERIC | 64)
#define EC_IOCTL_SET_SPLITTED_FRAME_PROCESSING_ENABLED          EC_IOCTL_SET_SPLIT_FRAME_PROCESSING_ENABLED /* obsolete */
#define EC_IOCTL_SET_ADJUST_CYCFRAMES_AFTER_SLAVES_STATE_CHANGE (EC_IOCTL_GENERIC | 65)

/********************************************************************************/
/** \brief Get Slave Statistics read period [ms] (EC_T_DWORD). 0: disable.
 * \note See EC_IOCTL_SET_SLVSTAT_PERIOD.
 * \param pbyInBuf          [in]  Should be set to EC_NULL
 * \param dwInBufSize       [in]  Should be set to 0
 * \param pbyOutBuf         [out] Slave Statistics read period [ms] (EC_T_DWORD)
 * \param dwOutBufSize      [in]  Size of the output buffer provided at pbyOutBuf in bytes
 * \param pdwNumOutData     [out] Pointer to EC_T_DWORD. Amount of bytes written to the output buffer.
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_GET_SLVSTAT_PERIOD                     (EC_IOCTL_GENERIC | 66)

/********************************************************************************/
/** \brief Enable or disable deferred EoE switching.
 * \param pbyInBuf          [in]  Pointer to value of EC_T_BOOL. EC_TRUE: Deferred EoE switching enabled.
 * \param dwInBufSize       [in]  Size of the input buffer provided at pbyInBuf in bytes
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_SET_EOE_DEFFERED_SWITCHING_ENABLED     (EC_IOCTL_GENERIC | 67)

/********************************************************************************/
/** \brief Force state change to INIT for all new slaves in the network after detection.
 * \param pbyInBuf          [in]  Pointer to EC_T_BOOL. EC_TRUE: Force state change, EC_FALSE: No state change.
 * \param dwInBufSize       [in]  Size of the input buffer provided at pbyInBuf in bytes
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_SET_NEW_BUSSLAVES_TO_INIT              (EC_IOCTL_GENERIC | 68)

/********************************************************************************/
/** \brief Set INPUTs to zero on WKC is zero.
 * \param pbyInBuf          [in]  Pointer to value of EC_T_BOOL. EC_TRUE: Set INPUTs to zero if WKC is zero.
 * \param dwInBufSize       [in]  Size of the input buffer provided at pbyInBuf in bytes
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_SET_ZERO_INPUTS_ON_WKC_ZERO            (EC_IOCTL_GENERIC | 69)

/********************************************************************************/
/** \brief Set INPUTs to zero on WKC error.
 * \param pbyInBuf          [in]  Pointer to value of EC_T_BOOL. EC_TRUE: INPUTs are set to zero on WKC error.
 * \param dwInBufSize       [in]  Size of the input buffer provided at pbyInBuf in bytes
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_SET_ZERO_INPUTS_ON_WKC_ERROR           (EC_IOCTL_GENERIC | 70)

/********************************************************************************/
/** \brief Change the mailbox polling interval.
 * \param pbyInBuf          [in]  Pointer to struct #EC_T_SET_MAILBOX_POLLING_CYCLES_DESC
 * \param dwInBufSize       [in]  Size of the input buffer in bytes, e.g. sizeof(EC_T_SET_MAILBOX_POLLING_CYCLES_DESC)
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_SET_MAILBOX_POLLING_CYCLES             (EC_IOCTL_GENERIC | 71)

/********************************************************************************/
/** \brief Enable or disable process data swapping according to SwapInfo of slave PDO in ENI file.
 * \param pbyInBuf          [in]  Pointer to EC_T_BOOL. EC_FALSE: SwapInfo handling enabled, EC_TRUE: SwapInfo handling disabled (ignored).
 * \param dwInBufSize       [in]  Size of the input buffer provided at pbyInBuf in bytes.
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_SET_IGNORE_SWAPDATA                    (EC_IOCTL_GENERIC | 72)

/********************************************************************************/
/** \brief Set maximal master state. emSetMasterState() returns with #EC_E_INVALIDSTATE if the requested master state exceeds the maximal master state.
 * \param pbyInBuf          [in]  Pointer to value of EC_T_STATE
 * \param dwInBufSize       [in]  Size of the input buffer in bytes, e.g. sizeof(EC_T_STATE)
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_SET_MASTER_MAX_STATE                           (EC_IOCTL_GENERIC | 73)
#define EC_IOCTL_GET_MASTER_MAX_STATE                           (EC_IOCTL_GENERIC | 74)
#define EC_IOCTL_SET_CONFIGDATA_MEMORY_POOL                     (EC_IOCTL_GENERIC | 75)

/********************************************************************************/
/** \brief Determines if the transition should stop with error if PDI watchdog is detected
 * \param pbyInBuf          [in]  Pointer to EC_T_BOOL
 * \param dwInBufSize       [in]  Size of the input buffer in bytes, e.g. sizeof(EC_T_BOOL)
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_SET_STOP_TRANSITION_ON_PDI_WATCHDOG            (EC_IOCTL_GENERIC | 76)

#define EC_IOCTL_SET_DIAGMSG_CODE_BASE                          (EC_IOCTL_GENERIC | 77)
#define EC_IOCTL_SET_BUS_DIAGNOSIS_COUNTERS_OVERFLOW_ENABLED    (EC_IOCTL_GENERIC | 78)
#define EC_IOCTL_SET_SENDCYCFRAMES_BEFORE_PROCESSALLRXFRAMES    (EC_IOCTL_GENERIC | 79)
#define EC_IOCTL_ADD_PADDING_TO_DC_FRAMES                       (EC_IOCTL_GENERIC | 80)

/********************************************************************************/
/** \brief Activates and set the size of the VoE receive FIFO.
 * \param pbyInBuf          [in]  Pointer to value of EC_T_WORD, size of the FIFO, use 0 to set it to the original size
 * \param dwInBufSize       [in]  Size of the input buffer in bytes, e.g. sizeof(EC_T_WORD)
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_ACTIVATE_VOE_RECV_FIFO                 (EC_IOCTL_GENERIC | 81)
#define EC_IOCTL_REGISTER_FRAMECALLBACK                 (EC_IOCTL_GENERIC | 82)

/********************************************************************************/
/** \brief Change the behavior when the configuration of the EtherCAT network is generated
 * according to a bus scan result of emConfigureNetwork() with the parameter
 * #eCnfType_GenPreopENI or  #eCnfType_GenOpENI.
 * In that case, default settings are taken to set e.g. the name of the EtherCAT slave device. The next table gives
 * an overview about the possible parameters to be changed.
 * \param pbyInBuf          [in]  Pointer to struct EC_T_SET_GEN_ENI_PARM
 * \param dwInBufSize       [in]  Size of the input buffer in bytes, e.g. sizeof(EC_T_SET_GEN_ENI_PARM)
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_SET_GEN_ENI_PARM                       (EC_IOCTL_GENERIC | 83)

/********************************************************************************/
/** \brief Reallocate the mailbox queues of the different mailbox protocols.
 * \param pbyInBuf          [in]  Pointer to struct EC_T_REALLOC_MBX_QUEUE_DESC
 * \param dwInBufSize       [in]  Size of the input buffer in bytes, e.g. sizeof(EC_T_REALLOC_MBX_QUEUE_DESC)
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_REALLOC_MBX_QUEUE                      (EC_IOCTL_GENERIC | 84)

/********************************************************************************/
/** \brief Set maximum state for the slave.
 * \param pbyInBuf          [in]  Pointer to struct EC_T_SLAVE_MAX_STATE_DESC
 * \param dwInBufSize       [in]  Size of the input buffer in bytes, e.g. sizeof(EC_T_SLAVE_MAX_STATE_DESC)
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_SET_SLAVE_MAX_STATE                    (EC_IOCTL_GENERIC | 85)

/********************************************************************************/
/** \brief This call determines if the link status is refreshed automatically.
 * \param pbyInBuf          [in]  Pointer to EC_T_BOOL
 * \param dwInBufSize       [in]  Size of the input buffer in bytes, e.g. sizeof(EC_T_BOOL)
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_SET_AUTO_REFRESH_LINK_STATUS           (EC_IOCTL_GENERIC | 86)

/********************************************************************************/
/** \brief Set inputs to zero on frame loss.
 * \param pbyInBuf          [in]  Pointer to value of EC_T_BOOL. EC_TRUE: Set inputs to zero on frame loss
 * \param dwInBufSize       [in]  Size of the input buffer provided at pbyInBuf in bytes
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_SET_ZERO_INPUTS_ON_FRAME_LOSS          (EC_IOCTL_GENERIC | 87)

/********************************************************************************/
/** \brief Enable clear error registers init cmd
 * \param pbyInBuf          [in]  Pointer to value of EC_T_BOOL. EC_TRUE: enable clear error registers init cmd
 * \param dwInBufSize       [in]  Size of the input buffer in bytes, e.g. sizeof(EC_T_BOOL)
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_SET_CLEAR_ERROR_REGISTERS_INITCMD_ENABLED      (EC_IOCTL_GENERIC | 88)

/********************************************************************************/
/** \brief This call specifies if the mailbox states count must be automatically adjusted according the mailbox state addresses in the ENI. This is needed if the ENI is inconsistent regarding the mailbox states, leading to Error 0x98130033 ENI: Inconsistent content. Default: Use ENI specified mailbox state count (EC_FALSE).
 * \param pbyInBuf          [in]  Pointer to EC_T_BOOL
 * \param dwInBufSize       [in]  Size of the input buffer in bytes, e.g. sizeof(EC_T_BOOL)
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_SET_AUTO_ADJUST_MBX_STATE_COUNT_ENABLED        (EC_IOCTL_GENERIC | 89)

/**
\brief Set the limit for Al Status Aging.
\param pbyInBuf          [in]  Pointer to value of EC_T_DWORD, limit for Al Stauts Aging, is 10 by default. 
\param dwInBufSize       [in]  Size of the input buffer in bytes, e.g. sizeof(EC_T_WORD)
\param pbyOutBuf         [out] Should be set to EC_NULL
\param dwOutBufSize      [in]  Should be set to 0
\param pdwNumOutData     [out] Should be set to EC_NULL
\return #EC_E_NOERROR or error code
*/
#define EC_IOCTL_SET_ALSTATUS_AGING_LIMIT       (EC_IOCTL_GENERIC | 90)

/**
\brief Add Latch Port Receive Times command to cyclic frame (NOP ADO 0x0900)
\param pbyInBuf          [in]  Should be set to EC_NULL
\param dwInBufSize       [in]  Should be set to 0
\param pbyOutBuf         [out] Should be set to EC_NULL
\param dwOutBufSize      [in]  Should be set to 0
\param pdwNumOutData     [out] Should be set to EC_NULL
\return #EC_E_NOERROR or error code
*/
#define EC_IOCTL_ADD_NOP_LATCH_PORT_RECEIVE_TIMES               (EC_IOCTL_GENERIC | 91)

/* Distributed Clocks (DC) */
#define EC_IOCTL_REG_DC_SLV_SYNC_NTFY                   (EC_IOCTL_DC |  3)
#define EC_IOCTL_UNREG_DC_SLV_SYNC_NTFY                 (EC_IOCTL_DC |  4)

/********************************************************************************/
/** \brief Get the last generated emNotify - EC_NOTIFY_DC_SLV_SYNC notification.
 * \param pbyInBuf          [in]  Should be set to EC_NULL
 * \param dwInBufSize       [in]  Should be set to 0
 * \param pbyOutBuf         [out] Pointer to EC_T_DC_SYNC_NTFY_DESC data type
 * \param dwOutBufSize      [in]  Size of the output buffer in bytes
 * \param pdwNumOutData     [out] Pointer to EC_T_DWORD. Amount of bytes written to the output buffer pbyOutBuf
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_DC_SLV_SYNC_STATUS_GET                 (EC_IOCTL_DC |  5)
#define EC_IOCTL_DC_SLV_SYNC_DEVLIMIT_SET               (EC_IOCTL_DC |  6)
#define EC_IOCTL_DC_SLV_SYNC_DEVLIMIT_GET               (EC_IOCTL_DC |  7)
#define EC_IOCTL_DC_SHIFT_SYSTIME                       (EC_IOCTL_DC | 16)

/********************************************************************************/
/** \brief Set the safety offset applied to the "set DC start time" InitCmd during the PS transition.
 * \param pbyInBuf          [in]  Pointer to EC_T_DC_STARTCYCSAFETY_DESC data type
 * \param dwInBufSize       [in]  Size of the input buffer provided at pbyInBuf in bytes
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_DC_SETSYNCSTARTOFFSET                  (EC_IOCTL_DC | 17)

/********************************************************************************/
/** \brief Enable or disable the usage of the first DC slave on bus overriding the configured reference clock.
 * \param pbyInBuf          [in]  Pointer to EC_T_BOOL. EC_FALSE: disable, EC_TRUE: enable
 * \param dwInBufSize       [in]  Size of the input buffer provided at pbyInBuf in bytes
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_DC_FIRST_DC_SLV_AS_REF_CLOCK           (EC_IOCTL_DC | 18)
#define EC_IOCTL_DC_SLAVE_CONTROLLED_BY_PDI             (EC_IOCTL_DC | 19)

/********************************************************************************/
/** \brief Enable or disable the usage of DC at all supporting slaves on bus overriding the configured settings.
 * Perhaps #EC_IOCTL_DC_FIRST_DC_SLV_AS_REF_CLOCK is necessary to set the reference clock at an allowed position.
 * \param pbyInBuf          [in]  Pointer to EC_T_BOOL. EC_FALSE: disable, EC_TRUE: enable
 * \param dwInBufSize       [in]  Size of the input buffer provided at pbyInBuf in bytes
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_DC_ENABLE_ALL_DC_SLV                   (EC_IOCTL_DC | 20)
#define EC_IOCTL_DC_SET_RED_PROPAGDELAY                 (EC_IOCTL_DC | 21)

/* DC Master Sync (DCM) */
#define EC_IOCTL_DCM_REGISTER_TIMESTAMP                 (EC_IOCTL_DCM |  1)
#define EC_IOCTL_DCM_UNREGISTER_TIMESTAMP               (EC_IOCTL_DCM |  2)
#define EC_IOCTL_DCM_REGISTER_STARTSO_CALLBACK          (EC_IOCTL_DCM |  3)

/********************************************************************************/
/** \brief Get logging information from the DCM controller.
 * \param pbyInBuf          [in]  Should be set to EC_NULL
 * \param dwInBufSize       [in]  Should be set to 0
 * \param pbyOutBuf         [out] Pointer to struct EC_T_DCM_LOG
 * \param dwOutBufSize      [in]  Size of the output buffer in bytes
 * \param pdwNumOutData     [out] Pointer to EC_T_DWORD. Amount of bytes written to the output buffer pbyOutBuf
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_DCM_GET_LOG                            (EC_IOCTL_DCM |  4)

/* Scan Bus (SB) */
/********************************************************************************/
/** \brief Start scanning the network (again). On completion the Notfication #EC_NOTIFY_SB_STATUS is given.
 * \param pbyInBuf          [in]  Should be set to EC_NULL
 * \param dwInBufSize       [in]  Should be set to 0
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_SB_RESTART                             (EC_IOCTL_SB |  1)           /* 0x00050001 */

/********************************************************************************/
/** \brief Get the status of the last bus scan.
 * \param pbyInBuf          [in]  Should be set to EC_NULL
 * \param dwInBufSize       [in]  Should be set to 0
 * \param pbyOutBuf         [out] Pointer to EC_T_SB_STATUS_NTFY_DESC
 * \param dwOutBufSize      [in]  Size of the output buffer in bytes
 * \param pdwNumOutData     [out] Pointer to EC_T_DWORD. Amount of bytes written to the output buffer.
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_SB_STATUS_GET                          (EC_IOCTL_SB |  2)           /* 0x00050002 */
#define EC_IOCTL_SB_SET_BUSCNF_VERIFY                   (EC_IOCTL_SB |  3)           /* 0x00050003 */
#define EC_IOCTL_SB_SET_BUSCNF_VERIFY_PROP              (EC_IOCTL_SB |  4)           /* 0x00050004 */
#define EC_IOCTL_SB_BUSCNF_GETSLAVE_INFO                (EC_IOCTL_SB |  5)           /* 0x00050005 */
#define EC_IOCTL_SB_BUSCNF_GETSLAVE_INFO_EEP            (EC_IOCTL_SB |  6)           /* 0x00050006 */

/********************************************************************************/
/** \brief Enables Busscan support. 
 * \param pbyInBuf          [in]  Pointer to Timeout Parameter Value [ms] (EC_T_DWORD). Timeout Parameter is used for timeout during Bus Topology determination.
 * \param dwInBufSize       [in]  Size of the input buffer provided at pbyInBuf in bytes
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_SB_ENABLE                              (EC_IOCTL_SB |  7)           /* 0x00050007 */
#define EC_IOCTL_SB_BUSCNF_GETSLAVE_INFO_EX             (EC_IOCTL_SB |  9)           /* 0x00050009 */

/********************************************************************************/
/** \brief Enables slave alias addressing for all slaves.
 * \param pbyInBuf          [in]  Should be set to EC_NULL
 * \param dwInBufSize       [in]  Should be set to 0
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_SLV_ALIAS_ENABLE                       (EC_IOCTL_SB | 10)           /* 0x0005000A */
#define EC_IOCTL_SB_SET_BUSCNF_READ_PROP                (EC_IOCTL_SB | 12)           /* 0x0005000C */

/********************************************************************************/
/** \brief Set the topology changed delay value. The master will wait this duration [ms] to react on appearing links in topology. The default value is 1000 ms.
 * \param pbyInBuf          [in]  Pointer to EC_T_DWORD containing the delay information [ms]
 * \param dwInBufSize       [in]  Size of the input buffer provided at pbyInBuf in bytes
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_SB_SET_TOPOLOGY_CHANGED_DELAY          (EC_IOCTL_SB | 13)           /* 0x0005000D */

/********************************************************************************/
/** \brief Enable or disable bus mismatch if IN and OUT connectors are swapped. If enabled the swapped IN and OUT connectors will lead to bus mismatch.
By default swapped IN and OUT connectors will lead to bus mismatch.
 * \param pbyInBuf          [in]  Pointer to EC_T_BOOL variable. If set to EC_TRUE swapped IN and OUT connectors will lead to bus mismatch, if set to EC_FALSE swapped IN and OUT connectors are tolerated.
 * \param dwInBufSize       [in]  Size of the input buffer provided at pbyInBuf in bytes
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_SB_SET_ERROR_ON_CROSSED_LINES          (EC_IOCTL_SB | 14)           /* 0x0005000E */

/********************************************************************************/
/** \brief Enable or disable the automatic topology change mode. By default the automatic mode is enabled.
 * \param pbyInBuf          [in]  Pointer to EC_T_BOOL variable. If set to EC_TRUE the automatic mode is enabled.
 * \param dwInBufSize       [in]  Size of the input buffer provided at pbyInBuf in bytes
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_SB_SET_TOPOLOGY_CHANGE_AUTO_MODE       (EC_IOCTL_SB | 15)           /* 0x0005000F */

/********************************************************************************/
/** \brief This call will trigger a scan bus for accept topology change (manual mode). On completion the Notfication #EC_NOTIFY_SB_STATUS is given.
 * \param pbyInBuf          [in]  Should be set to EC_NULL
 * \param dwInBufSize       [in]  Should be set to 0
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_SB_ACCEPT_TOPOLOGY_CHANGE              (EC_IOCTL_SB | 16)           /* 0x00050010 */

/********************************************************************************/
/** \brief Enable or disable unexpected bus slaves notification.
 * \param pbyInBuf          [in]  Pointer to EC_T_BOOL variable. If set to EC_TRUE unexpected bus slaves on the network will be notified by #EC_NOTIFY_SB_MISMATCH.
 * \param dwInBufSize       [in]  Size of the input buffer provided at pbyInBuf in bytes
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_SB_NOTIFY_UNEXPECTED_BUS_SLAVES        (EC_IOCTL_SB | 17)           /* 0x00050011 */

/********************************************************************************/
/** \brief Enable or disable the enhanced line crossed detection if redundancy is configured.
 * \param pbyInBuf          [in]  Pointer to EC_T_BOOL. If value is EC_TRUE enhanced line crossed detection is enabled, if EC_FALSE it is not.
 * \param dwInBufSize       [in]  Should be set to sizeof(EC_T_BOOL).
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_SB_SET_RED_ENHANCED_LINE_CROSSED_DETECTION_ENABLED (EC_IOCTL_SB | 18)           /* 0x00050012 */
#define EC_IOCTL_SB_SET_NOTIFY_NOT_CONNECTED_PORT_A                 (EC_IOCTL_SB | 19)           /* 0x00050013 */
#define EC_IOCTL_SB_SET_NOTIFY_UNEXPECTED_CONNECTED_PORT            (EC_IOCTL_SB | 20)           /* 0x00050014 */

/********************************************************************************/
/** \brief Set junction redundancy mode.
 * \param pbyInBuf          [in]  Pointer to value of EC_T_JUNCTION_REDUNDANCY_MODE
 * \param dwInBufSize       [in]  Should be set to sizeof(EC_T_JUNCTION_REDUNDANCY_MODE)
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_SB_SET_JUNCTION_REDUNDANCY_MODE        (EC_IOCTL_SB | 21)           /* 0x00050015 */
#define EC_IOCTL_SB_SET_JUNCTION_REDUNDANCY_ENABLED     EC_IOCTL_SB_SET_JUNCTION_REDUNDANCY_MODE /* obsolete */
#define EC_IOCTL_SB_GET_BUS_SLAVE_PORTS_INFO            (EC_IOCTL_SB | 22)           /* 0x00050016 */

/********************************************************************************/
/** \brief Enable or disable bus mismatch if a line is broken in a redundant network. If enabled, line breaks in cable or junction redundant networks will lead to bus mismatch.
 * \note #EC_E_REDLINEBREAK: Line break in a cable redundant network.
 * \note #EC_E_JUNCTION_RED_LINE_BREAK: Line break in a junction redundant network.
 * \note See #EC_NOTIFY_SB_MISMATCH, #EC_NOTIFY_SB_STATUS.
 * \param pbyInBuf          [in]  Pointer to value of EC_T_BOOL: EC_TRUE: Enable, EC_FALSE: Disable. Default: Enabled.
 * \param dwInBufSize       [in]  Should be set to sizeof(EC_T_BOOL)
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_SB_SET_ERROR_ON_LINE_BREAK             (EC_IOCTL_SB | 23)           /* 0x00050017 */

#define EC_IOCTL_SB_SET_IDENTIFICATION_FALLBACK_ENABLED (EC_IOCTL_SB | 24)           /* 0x00050018 */

/********************************************************************************/
/** \brief Declares that no DC slaves are located after junction.
 * \param pbyInBuf          [in]  Pointer to EC_T_BOOL variable. If set to EC_TRUE the hidden slave detection and the junction redundancy specific propagation delay measurement are not executed.
 * \param dwInBufSize       [in]  Size of the input buffer provided at pbyInBuf in bytes
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_SB_SET_NO_DC_SLAVES_AFTER_JUNCTION     (EC_IOCTL_SB | 25)           /* 0x00050019 */

/********************************************************************************/
/** \brief Set the topology changed delay values individually. The master will wait individual durations [ms] (0 ms: disabled) for slave ports, main link and red link to react on appearing links in topology. The default value is 1000 ms.
 * \param pbyInBuf          [in]  Pointer to EC_T_TOPOLOGY_CHANGED_DELAYS containing the delay durations [ms]
 * \param dwInBufSize       [in]  Size of the input buffer provided at pbyInBuf in bytes
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_SB_SET_TOPOLOGY_CHANGED_DELAYS         (EC_IOCTL_SB | 26)           /* 0x0005001A */

/* Hot Connect (HC) */

/********************************************************************************/
/** \brief Configures the Hot-Connect mode. Can be called at any time after emInitMaster(), usually before emConfigureNetwork(), but not necessarily. If it is not called, the Hot-Connect instance operates in automatic mode EC_T_EHOTCONNECTMODE::echm_automatic.
 * \param pbyInBuf          [in]  Pointer to EC_T_EHOTCONNECTMODE
 * \param dwInBufSize       [in]  sizeof(EC_T_EHOTCONNECTMODE)
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_HC_SETMODE                             (EC_IOCTL_HC | 1)

/********************************************************************************/
/** \brief Get the current Hot-Connect operating mode.
 * \param pbyInBuf          [in]  Should be set to EC_NULL
 * \param dwInBufSize       [in]  Should be set to 0
 * \param pbyOutBuf         [out] Pointer to EC_T_EHOTCONNECTMODE
 * \param dwOutBufSize      [in]  sizeof(EC_T_EHOTCONNECTMODE)
 * \param pdwNumOutData     [out] Pointer to DWORD value carrying sizeof(EC_T_EHOTCONNECTMODE)
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_HC_GETMODE                             (EC_IOCTL_HC | 2)

/********************************************************************************/
/** \brief Configures the Timeout values used by Hot-Connect.
 * \param pbyInBuf          [in]  Pointer to EC_T_HC_CONFIGURETIMEOUTS_DESC
 * \param dwInBufSize       [in]  sizeof(EC_T_HC_CONFIGURETIMEOUTS_DESC)
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_HC_CONFIGURETIMEOUTS                   (EC_IOCTL_HC | 3)

/* Simulator */

/********************************************************************************/
/** \brief Enable or disable mailbox processing at slave. Default: Enabled.
 * \param pbyInBuf          [in]  Pointer to value of #EC_T_SIMULATOR_MBX_PROCESS_CTL_DESC
 * \param dwInBufSize       [in]  Size of the input buffer provided at pbyInBuf in bytes
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_SIMULATOR_SET_MBX_PROCESS_CTL          (EC_IOCTL_SIMULATOR | 1)     /* 0xCC000001 */

/********************************************************************************/
/** \brief Get the information if mailbox processing at slave is enabled or disabled.
 * \param pbyInBuf         [in]  Pointer to value of EC_T_WORD. Configured station address of slave.
 * \param dwInBufSize      [in]  Size of the input buffer provided at pbyInBuf in bytes
 * \param pbyOutBuf        [out] Pointer to value of #EC_T_SIMULATOR_MBX_PROCESS_CTL_DESC
 * \param dwOutBufSize     [in]  Size of the output buffer provided at pbyOutBuf in bytes
 * \param pdwNumOutData    [out] Pointer to EC_T_DWORD. Amount of bytes written to the output buffer.
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_SIMULATOR_GET_MBX_PROCESS_CTL          (EC_IOCTL_SIMULATOR | 2)     /* 0xCC000002 */

/* Monitor */

/********************************************************************************/
/** \brief Enable or disable clear on read of the CoE SDO data. If clear on read is enabled, the data is automatically cleared after each read of the CoE SDO Index, SubIndex using : emonCoeSdoUpload() or : emonCoeSdoUploadReq(). The IOCTL must be called after : emonConfigureNetwork().
 * \param pbyInBuf         [in]  Pointer to value of EC_T_BOOL. EC_TRUE: enable clear on read
 * \param dwInBufSize      [in]  Size of the input buffer provided at pbyInBuf in bytes
 * \param pbyOutBuf        [out] Should be set to EC_NULL
 * \param dwOutBufSize     [in]  Should be set to 0
 * \param pdwNumOutData    [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_MONITOR_SET_COESDO_CLEAR_ON_READ           (EC_IOCTL_MONITOR | 1) /* 0xCD000001 */

#define EC_IOCTL_MONITOR_SET_IGNORE_COE_API_TIMEOUTS        (EC_IOCTL_MONITOR | 2) /* 0xCD000002 */
/* private (PRIVATE) */

/********************************************************************************/
/** \brief This IO Control enables the application to simulate the loss of sent and/or received EtherCAT frames for testing purposes.
    Three modes of operation are possible: Random, periodic or random periodic frame loss simulation.
    - Random frame loss simulation: For each frame the dwFrameLossLikelihoodPpm parameter determines whether the frame will be discarded.
    - Periodic frame loss simulation: After dwFixedLossNumLostFrames discarded frames, dwFixedLossNumGoodFrames frames will be processed.
    - Random periodic frame loss simulation: The dwFrameLossLikelihoodPpm parameter determines whether a periodic frame loss sequence is triggered.
 * \param pbyInBuf          [in]  Array of four EC_T_DWORDs (arrDword)
 * \param dwInBufSize       [in]  Size of the input buffer provided at pbyInBuf in bytes
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_SET_FRAME_LOSS_SIMULATION              (EC_IOCTL_PRIVATE | 1)

/********************************************************************************/
/** \brief Same as #EC_IOCTL_SET_FRAME_LOSS_SIMULATION but only enables receive direction frame losses.
 * \param pbyInBuf          [in]  Should be set to EC_NULL
 * \param dwInBufSize       [in]  Should be set to 0
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_SET_RXFRAME_LOSS_SIMULATION            (EC_IOCTL_PRIVATE | 2)

/********************************************************************************/
/** \brief Same as #EC_IOCTL_SET_FRAME_LOSS_SIMULATION but only enables transmit direction frame losses.
 * \param pbyInBuf          [in]  Should be set to EC_NULL
 * \param dwInBufSize       [in]  Should be set to 0
 * \param pbyOutBuf         [out] Should be set to EC_NULL
 * \param dwOutBufSize      [in]  Should be set to 0
 * \param pdwNumOutData     [out] Should be set to EC_NULL
 * \return #EC_E_NOERROR or error code
 */
#define EC_IOCTL_SET_TXFRAME_LOSS_SIMULATION            (EC_IOCTL_PRIVATE | 3)
#define EC_IOCTL_GET_FAST_CONTEXT                       (EC_IOCTL_PRIVATE | 4)
#define EC_IOCTL_SET_OEM_KEY                            (EC_IOCTL_PRIVATE | 5)
#define EC_IOCTL_CHECK_OEM_KEY                          (EC_IOCTL_PRIVATE | 6)

/********************************************************************************/
/** \typedef EC_T_PFMEMREQ
 * \param [in]  pvContext   Arbitrarily application-defined parameter passed to callback
 * \param [in]  dwTaskId    Task ID of cyclic data transfer (ENI: Cyclic/TaskId). If TASKID_COMPLETE_PD is given, the function must return a complete output process data buffer which contains valid data for all cyclic tasks.
 * \param [out] ppbyPDData  Pointer to the process data buffer to be used. If set to EC_NULL, the corresponding fixed buffer from EC_T_MEMPROV_DESC is used. The provided buffer size must correspond to the caller context.
 */
typedef EC_T_VOID (EC_FNCALL *EC_T_PFMEMREQ)(EC_T_PVOID pvContext, EC_T_DWORD dwTaskId, EC_T_PBYTE* ppbyPDData);

/********************************************************************************/
/** \typedef EC_T_PFMEMREL
 * \param [in]  pvContext   Arbitrarily application-defined parameter passed to callback
 * \param [in]  dwTaskId    Task ID of cyclic data transfer (ENI: Cyclic/TaskId)
 */
typedef EC_T_VOID (EC_FNCALL *EC_T_PFMEMREL)(EC_T_PVOID pvContext, EC_T_DWORD dwTaskId);

/* Descriptor for EC_IOCTL_REGISTER_PDMEMORYPROVIDER */
typedef struct _EC_T_MEMPROV_DESC
{
    EC_T_PVOID      pvContext;                      /**< Context pointer. This pointer is used every time one of the callback functions (e.g. pfPDOutReadRequest) is called. */
    EC_T_PBYTE      pbyPDOutData;                   /**< Pointer to the fixed output process data buffer (values transferred from the master to the slaves).
                                                         A value of EC_NULL may be given in case the pointer will be provided later when the function EC_T_MEMPROV_DESC.pfPDOutDataReadRequest is called. */
    EC_T_DWORD      dwPDOutDataLength;              /**< Length of the output process data buffer */
    EC_T_PBYTE      pbyPDInData;                    /**< Pointer to the fixed input process data buffer (values transferred from the slaves to the master).
                                                         A value of EC_NULL may be given in case the pointer will be provided later when the function EC_T_MEMPROV_DESC.pfPDInDataWriteRequest is called. */
    EC_T_DWORD      dwPDInDataLength;               /**< Length of the input process data buffer */
    EC_T_PFMEMREQ   pfPDOutDataReadRequest;         /**< This function will be called cyclically within the process data transfer cycle prior to reading data from the output process data buffer.
                                                         If EC_NULL is set, the fixed buffer EC_T_MEMPROV_DESC.pbyPDOutData is used. */
    EC_T_PFMEMREL   pfPDOutDataReadRelease;         /**< This function will be called cyclically within the process data transfer cycle after all data were read from the output process data buffer */

    EC_T_PFMEMREQ   pfPDOutDataWriteRequest;        /**< This function will be called cyclically within the process data transfer cycle prior to writing new data into the output process data buffer.
                                                         If EC_NULL is set, the fixed buffer EC_T_MEMPROV_DESC.pbyPDOutData is used. */
    EC_T_PFMEMREL   pfPDOutDataWriteRelease;        /**< This function will be called cyclically within the process data transfer cycle after all data were written into the output process data buffer */

    EC_T_PFMEMREQ   pfPDInDataWriteRequest;         /**< This function will be called cyclically within the process data transfer cycle prior to writing new data into the input process data buffer.
                                                         If EC_NULL is set, the fixed buffer EC_T_MEMPROV_DESC.pbyPDInData is used. */
    EC_T_PFMEMREL   pfPDInDataWriteRelease;         /**< This function will be called cyclically within the process data transfer cycle after all data were written into the input process data buffer */

    EC_T_PBYTE      pbyMasterRedPDOutData;          /**< Pointer to the MasterRed output process data buffer (ACTIVE to INACTIVE) */
    EC_T_DWORD      dwMasterRedPDOutDataLength;     /**< Length of the MasterRed output process data buffer */
    EC_T_PBYTE      pbyMasterRedPDInData;           /**< Pointer to the default input process data buffer (INACTIVE to ACTIVE) */
    EC_T_DWORD      dwMasterRedPDInDataLength;      /**< Length of the input process data buffer */
    EC_T_PFMEMREQ   pfMasterRedPDOutReadRequest;    /**< This function will be called within the process data transfer cycle prior to reading data */
    EC_T_PFMEMREL   pfMasterRedPDOutReadRelease;    /**< This function will be called after all data has been read from the output process data buffer */
    EC_T_PFMEMREQ   pfMasterRedPDOutWriteRequest;   /**< This function will be called within the process data transfer cycle prior to reading data */
    EC_T_PFMEMREL   pfMasterRedPDOutWriteRelease;   /**< This function will be called after all data were read from the output process data buffer */
    EC_T_PFMEMREQ   pfMasterRedPDInWriteRequest;    /**< This function will be called within the process data transfer cycle prior to writing data */
    EC_T_PFMEMREL   pfMasterRedPDInWriteRelease;    /**< This function will be called after all data were written to the input process data buffer */
    EC_T_PFMEMREQ   pfMasterRedPDInReadRequest;     /**< This function will be called within the process data transfer cycle prior to writing data */
    EC_T_PFMEMREL   pfMasterRedPDInReadRelease;     /**< This function will be called after all data were written to the input process data buffer */
} EC_PACKED(4) EC_T_MEMPROV_DESC, *EC_PT_MEMPROV_DESC;

/* Descriptor for EC_IOCTL_GET_CYCLIC_CONFIG_INFO */
typedef struct _EC_T_CYC_CONFIG_DESC
{
    EC_T_DWORD      dwNumCycEntries;    /**< [out] Total number of cyclic entries */
    EC_T_DWORD      dwTaskId;           /**< [out] Task ID of selected cyclic entry (ENI: Cyclic/TaskId) */
    EC_T_DWORD      dwPriority;         /**< [out] Priority of selected cyclic entry */
    EC_T_DWORD      dwCycleTime;        /**< [out] Cycle time of selected cyclic entry */
} EC_PACKED(4) EC_T_CYC_CONFIG_DESC;

typedef struct _EC_T_SLVSTATISTICS_DESC
{
    EC_T_BYTE       abyInvalidFrameCnt[ESC_PORT_COUNT]; /**< [out] Invalid Frame Counters per Slave Port */
    EC_T_BYTE       abyRxErrorCnt[ESC_PORT_COUNT];      /**< [out] RX Error Counters per Slave Port */
    EC_T_BYTE       abyFwdRxErrorCnt[ESC_PORT_COUNT];   /**< [out] Forwarded RX Error Counters per Slave Port */
    EC_T_BYTE       byProcessingUnitErrorCnt;           /**< [out] Processing Unit Error Counter */
    EC_T_BYTE       byPdiErrorCnt;                      /**< [out] PDI Error Counter */
    EC_T_WORD       wAlStatusCode;                      /**< [out] AL Status Code */
    EC_T_BYTE       abyLostLinkCnt[ESC_PORT_COUNT];     /**< [out] Lost Link Counters per Slave Port */

    EC_T_UINT64     qwReadTime;                         /**< [out] Timestamp of the last read [ns] */
    EC_T_UINT64     qwChangeTime;                       /**< [out] Timestamp of the last counter change [ns] */
} EC_PACKED(4) EC_T_SLVSTATISTICS_DESC;

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_TOPOLOGY_CHANGED_DELAYS
{
    EC_T_DWORD      dwSlavePort;                    /**< [in] Delay before opening slave port after link connection detected */
    EC_T_DWORD      dwMainLine;                     /**< [in] Delay before sending frames at main line after link connection detected */
    EC_T_DWORD      dwRedLine;                      /**< [in] Delay before sending frames at red line after link connection detected */

    EC_T_DWORD      adwReserved[5];                 /**< reserved */
} EC_PACKED_API  EC_T_TOPOLOGY_CHANGED_DELAYS;
#include EC_PACKED_INCLUDESTOP

/* Descriptor for EC_IOCTL_GET_PDMEMORYSIZE */
#include EC_PACKED_INCLUDESTART(1)
typedef struct _EC_T_MEMREQ_DESC
{
    EC_T_DWORD  dwPDOutSize;                        /**< Size of the output process data image */
    EC_T_DWORD  dwPDInSize;                         /**< Size of the input process data image */
} EC_PACKED(1) EC_T_MEMREQ_DESC;
#include EC_PACKED_INCLUDESTOP

/* Descriptor for EC_IOCTL_SET_MASTER_DEFAULT_TIMEOUTS */
#include EC_PACKED_INCLUDESTART(1)
typedef struct _EC_T_MASTERDEFAULTTIMEOUTS_DESC
{
    EC_T_DWORD  dwMasterStateChange;                /**< Default state change timeout [ms], applied if emSetMasterState called with EC_NOWAIT */
    EC_T_DWORD  dwInitCmdRetry;                     /**< Timeout [ms] between retry sending an init-command */
    EC_T_DWORD  dwMbxCmd;                           /**< Timeout [ms] between retry sending an mailbox command */
    EC_T_DWORD  dwMbxPolling;                       /**< Mailbox polling cycle [ms] \defaultparm{EC_DEFAULTPARM_dwMbxPollingTime} */
    EC_T_DWORD  dwDcmInSync;                        /**< Timeout [ms] to wait for DCM InSync in state change PREOP to SAFEOP \defaultparm{EC_DEFAULTPARM_dwDcmInSyncTimeout} */
    EC_T_WORD   wInitCmd;                           /**< Timeout [ms] to InitCmds if not specified in ENI \defaultparm{EC_DEFAULTPARM_wInitCmdTimeout} */
    EC_T_WORD   wReserved;
    EC_T_DWORD  dwSlaveIdentification;              /**< Timeout [ms] to wait for the reading of the slave identification \defaultparm{EC_DEFAULTPARM_dwSlaveIdentificationTimeout} */
    EC_T_DWORD  dwGenerateEni;                      /**< Timeout [ms] to wait for eCnfType_GenPreopENI, eCnfType_GenOpENI, eCnfType_Gen... \defaultparm{EC_DEFAULTPARM_dwGenerateEniTimeout} */
    EC_T_DWORD  dwBootMbxPolling;                   /**< Mailbox polling cycle [ms] for Boot \defaultparm{EC_DEFAULTPARM_dwMbxPollingTime} */
    EC_T_DWORD  dwReserved[7];
} EC_PACKED(1) EC_T_MASTERDEFAULTTIMEOUTS_DESC;
#include EC_PACKED_INCLUDESTOP

/********************************************************************************/
/** \typedef EC_PF_CYCFRAME_RECV
 * \param dwTaskId     [in] Task id of the received cyclic frame
 * \param pvContext    [in] Context pointer. This pointer is used as parameter every time when the callback function is called.
 */
typedef EC_T_VOID (EC_FNCALL *EC_PF_CYCFRAME_RECV)(EC_T_DWORD dwTaskId, EC_T_VOID* pvContext);

#include EC_PACKED_INCLUDESTART(1)
typedef struct _EC_T_CYCFRAME_RX_CBDESC
{
    EC_T_VOID*          pCallbackContext;   /**< [in] Context pointer. This pointer is used as parameter every time the callback function is called. */
    EC_PF_CYCFRAME_RECV pfnCallback;        /**< [in] This function will be called after the cyclic frame is received, if there is more than one cyclic frame after the last frame. The application has to assure that these functions will not block. */
} EC_PACKED(1)  EC_T_CYCFRAME_RX_CBDESC;
#include EC_PACKED_INCLUDESTOP

typedef enum _EC_T_CYCFRAME_LAYOUT
{
    eCycFrameLayout_STANDARD    = 0,    /**< Layout according ENI with command add/reordering, no relationship to PD */
    eCycFrameLayout_DYNAMIC     = 1,    /**< Layout is dynamically modified to send as few as possible cyclic frames and commands */
    eCycFrameLayout_FIXED       = 2,    /**< Layout strictly match ENI, frame buffers and PD area overlapped */
    eCycFrameLayout_IN_DMA      = 3,    /**< Layout strictly match ENI, frame buffers and PD area overlapped, frame buffers in DMA */

    eCycFrameLayout_BCppDummy   = 0xFFFFFFFF
} EC_T_CYCFRAME_LAYOUT;

#include EC_PACKED_INCLUDESTART(1)
typedef struct _EC_T_SET_NOTIFICATION_ENABLED_PARMS
{
    EC_T_DWORD dwClientId;          /**< [in] Client ID, 0: Master */
    EC_T_DWORD dwCode;              /**< [in] Notification code or #EC_ALL_NOTIFICATIONS */
    EC_T_DWORD dwEnabled;           /**< [in] Enable, disable or reset to default notification. See \ref EC_SET_NOTIFICATION_ENABLED "EC_NOTIFICATION_" flags. */
} EC_PACKED(1) EC_T_SET_NOTIFICATION_ENABLED_PARMS;
#include EC_PACKED_INCLUDESTOP

#include EC_PACKED_INCLUDESTART(1)
typedef struct _EC_T_GET_NOTIFICATION_ENABLED_PARMS
{
    EC_T_DWORD dwClientId;          /**< [in] Client ID, 0: Master */
    EC_T_DWORD dwCode;              /**< [in] Notification code */
} EC_PACKED(1) EC_T_GET_NOTIFICATION_ENABLED_PARMS;
#include EC_PACKED_INCLUDESTOP

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_CLEAR_MASTER_INFO_COUNTERS_PARMS
{
    EC_T_DWORD  dwClearBusDiagnosisCounters;        /**< [in] Bit 0..7: Clear corresponding Counter ID:
                                                        - Bit 0: Clear all Counters
                                                        - Bit 1: Clear Tx Frame Counter
                                                        - Bit 2: Clear Rx Frame Counter
                                                        - Bit 3: Clear Lost Frame Counter
                                                        - Bit 4: Clear Cyclic Frame Counter
                                                        - Bit 5: Clear Cyclic Datagram Counter
                                                        - Bit 6: Clear Acyclic Frame Counter
                                                        - Bit 7: Clear Acyclic DataGram Counter 
                                                        - Bit 8: Clear Cyclic Lost Frame Counter 
                                                        - Bit 9: Clear Acyclic Lost Frame Counter */
    EC_T_UINT64 qwMailboxStatisticsClearCounters;   /**< [in]  Bit 0..56: Clear corresponding Counter ID.
                                                        - Bit 0..7: Clear AoE statistics
                                                            - Bit 0: Total Read Transfer Count
                                                            - Bit 1: Read Transfer Count Last Second
                                                            - Bit 2: Total Bytes Read
                                                            - Bit 3: Bytes Read Last Second
                                                            - Bit 4: Total Write Transfer Count
                                                            - Bit 5: Write Transfer Count Last Second
                                                            - Bit 6: Total Bytes Write
                                                            - Bit 7: Bytes Write Last Second
                                                        - Bit 8..15: Clear CoE statistics (same ordering as Bit 0..7, AoE)
                                                        - Bit 16..23: Clear EoE statistics (same ordering as Bit 0..7, AoE)
                                                        - Bit 24..31: Clear FoE statistics (same ordering as Bit 0..7, AoE)
                                                        - Bit 32..39: Clear SoE statistics (same ordering as Bit 0..7, AoE)
                                                        - Bit 40..47: Clear VoE statistics (same ordering as Bit 0..7, AoE)
                                                        - Bit 48..55: Clear RawMbx statistics (same ordering as Bit 0..7, AoE)*/
    EC_T_DWORD  dwReserved[6];
} EC_PACKED_API EC_T_CLEAR_MASTER_INFO_COUNTERS_PARMS;
#include EC_PACKED_INCLUDESTOP

/* Descriptor for EC_IOCTL_SET_MAILBOX_POLLING_CYCLES */
#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_SET_MAILBOX_POLLING_CYCLES_DESC
{
    EC_T_DWORD      dwSlaveId;          /**< [in] Slave Id */
    EC_T_WORD       wCycles;            /**< [in] Mailbox polling interval [ms] */
} EC_PACKED_API EC_T_SET_MAILBOX_POLLING_CYCLES_DESC;
#include EC_PACKED_INCLUDESTOP

#define EC_GEN_ENI_PARM_ID_SLAVE_PREFIX          1
#define EC_GEN_ENI_PARM_ID_IGNORE_SCANBUS_ERRROR 2
#define EC_GEN_ENI_PARM_ID_SLAVE_ADDRESS         3
#include EC_PACKED_API_INCLUDESTART
typedef union _EC_T_GEN_ENI_PARM
{
    EC_T_CHAR szSlaveNamePrefix[MAX_SHORT_STRLEN + 1];  /**< Prefix to the EtherCAT slave device name and its variable names \defaultparm{EC_DEFAULTPARM_szSlaveNamePrefix} */    
    EC_T_BOOL bIgnoreScanBusError;                      /**< EC_TRUE ignore ScanBus error, EC_FALSE abort and return ScanBus status on error \defaultparm{EC_DEFAULTPARM_bIgnoreScanBusError} */
    EC_T_WORD wSlaveAddress;                            /**< Start address of the slave station adresses \defaultparm{EC_DEFAULTPARM_wSlaveAddress} */
} EC_PACKED_API EC_T_GEN_ENI_PARM;
#include EC_PACKED_INCLUDESTOP

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_SET_GEN_ENI_PARM
{
    EC_T_DWORD dwParmId;                          /**< ID of parameter to be set (EC_GEN_ENI_PARM_ID_...) */

    EC_PACKED_API_MEMBER \
        EC_T_GEN_ENI_PARM GenEniParm;             /**< Value of parameter to be set */
} EC_PACKED_API EC_T_SET_GEN_ENI_PARM;
#include EC_PACKED_INCLUDESTOP

#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_REALLOC_MBX_QUEUE_DESC
{
    EC_T_WORD       wSlaveFixedAddress;    /**< Slave fixed address, 0 for all slaves */
    EC_T_WORD       wMbxProtocols;         /**< Combination of Mbx protocols EC_MBX_PROTOCOL_COE... */
    EC_T_DWORD      dwMaxMbxTferQueued;    /**< Maximal number of transfers queued for the specified mailbox protocol of the specified slave */
}  EC_PACKED_API EC_T_REALLOC_MBX_QUEUE_DESC;
#include EC_PACKED_INCLUDESTOP

/* Descriptor for EC_IOCTL_SIMULATOR_SET_MBX_PROCESS_CTL / EC_IOCTL_SIMULATOR_GET_MBX_PROCESS_CTL */
#include EC_PACKED_API_INCLUDESTART
typedef struct _EC_T_SIMULATOR_MBX_PROCESS_CTL_DESC
{
    EC_T_WORD       wCfgFixedAddress;           /**< Slave's station address. 0: all slaves. */
    EC_T_BOOL       bReadMbxOutEnabled;         /**< Read mailbox out sync manager from DPRAM (received mailbox data from the master) */
    EC_T_BOOL       bProcessMbxEnabled;         /**< Process mailbox data from the master (MailboxServiceInd) */
} EC_PACKED_API EC_T_SIMULATOR_MBX_PROCESS_CTL_DESC;
#include EC_PACKED_INCLUDESTOP
#endif /* INC_ECIOCTL */

/*-END OF SOURCE FILE--------------------------------------------------------*/
