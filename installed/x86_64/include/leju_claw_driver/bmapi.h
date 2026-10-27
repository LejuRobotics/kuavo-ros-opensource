/**
 * @file        bmapi.h
 * @brief       Busmust device communication API.
 * @author      busmust
 * @version     1.10.2.30
 * @copyright   Copyright 2020 by Busmust Tech Co.,Ltd <br>
 *              All rights reserved. Property of Busmust Tech Co.,Ltd.<br>
 *              Restricted rights to use, duplicate or disclose of this code are granted through contract.
 */
#ifndef __BMAPI_H__
#define __BMAPI_H__

#include <stdint.h>
#undef c  
#include "bm_usb_def.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @def   BM_API_VERSION
 * @brief API version, format: major.minor.revision.build
 * @note  This macro might be checked by the application so that it could adapt to different versions of BMAPI.
 */
#define BM_API_VERSION         0x1a00021e

/**
 * @def   BMAPI
 * @brief All function declared with BMAPI modifier are exported by BMAP dynamic library (*.dll, *.so).
 */
#ifdef BMAPI_EXPORT
#ifdef _MSC_VER
#define BMAPI   __declspec(dllexport)
#else
#define BMAPI   __attribute__((visibility("default")))
#endif
#else
#ifdef _MSC_VER
#define BMAPI   __declspec(dllimport)
#else
#define BMAPI
#endif
#endif

/**
 * @typedef BM_ChannelHandle
 * @brief   Abstract handle to opened Busmust device channel, 
 *          most APIs would require a handle to operate on the target device channel.
 */
typedef void* BM_ChannelHandle;

/**
 * @typedef BM_NotificationHandle
 * @brief   Abstract handle to notification event of opened Busmust device channel, 
 *          call BM_WaitForNotifications to wait for new events (i.e. CAN RX message event).
 */
typedef void* BM_NotificationHandle;

/**
 * @typedef BM_AutosetCallbackHandle
 * @brief   Pointer to a Callback function when AUTOSET status is updates, indicating a bitrate option has passed or failed.
 * @param[in] bitrate      The bitrate option value which has passed or failed.
 * @param[in] tres         The terminal resistor option value which has passed or failed.
 * @param[in] nrxmessages  Number of received messages while listening to the bus using bitrate and tres.
 * @param[in] userarg      Arbitrary user argument passed by BM_Autoset().
 */
typedef void (*BM_AutosetCallbackHandle)(const BM_BitrateTypeDef* bitrate, BM_TerminalResistorTypeDef tres, int nrxmessages, uintptr_t userarg);

/**
 * @brief  Initialize BMAPI library, this function shall be called before any other API calls and shall only be called once.
 * @return Operation status, see BM_StatusTypeDef for details.
 */
BMAPI BM_StatusTypeDef BM_Init(void);

/**
 * @brief  Un-initialize BMAPI library, this function shall be called after any other API calls and shall only be called once.
 * @return Operation status, see BM_StatusTypeDef for details.
 */
BMAPI BM_StatusTypeDef BM_UnInit(void);

/**
 * @brief        Enumerate all connected Busmust device.
 * @param[out]   channelinfos  An array of BM_ChannelInfoTypeDef structure which holds info of all the enumerated Busmust devices.
 * @param[inout] nchannels     Number of device channels available, which is also the number of valid entries in channelinfos, 
 *                             this param must be initialized with the maximum length of the channelinfos array when calling this function.
 * @return       Operation status, see BM_StatusTypeDef for details.
 */
BMAPI BM_StatusTypeDef BM_Enumerate(BM_ChannelInfoTypeDef channelinfos[], int* nchannels);

/**
 * @brief Start AUTOSET sequence, for a BM-USB-CAN(FD) device, the AUTOSET sequence will detect the correct bitrate and terminal resistor.
 * @param[in]  channelinfo  Info of the device channel to run AUTOSET on, usually the info is filled by BM_Enumerate().
 * @param[out] bitrate      The detected bitrate.
 * @param[out] tres         The detected terminal resistor.
 * @param[in]  callback     A callback function which will be called on each step of AUTOSET.
 * @param[in]  userarg      Arbitrary user argument of the callback function, this argument will be passed to the callback as is.
 */
BMAPI BM_StatusTypeDef BM_Autoset(
    BM_ChannelInfoTypeDef* channelinfo,
    BM_BitrateTypeDef* bitrate,
    BM_TerminalResistorTypeDef* tres,
    BM_AutosetCallbackHandle callback,
    uintptr_t userarg
);

/**
 * @brief Open the specified CAN device port.
 * @param[in] port  Index of the port, starting from zero, note this is the index of all enumerated ports.
 * @return Handle to the opened CAN device channel, return NULL if failed to open the specified port.
 */
BMAPI BM_ChannelHandle BM_OpenCan(uint16_t port);

/**
 * @brief Open the specified device port using given configuration.
 * @param[out] handle      Handle to the opened device channel.
 * @param[in]  channelinfo Info of the device channel to open, usually the info is filled by BM_Enumerate().
 * @param[in]  mode        CAN operation mode option of the opened channel, see BM_CanModeTypeDef for details.
 * @param[in]  tres        Terminal resistor option of the opened channel, see BM_TerminalResistorTypeDef for details.
 * @param[in]  bitrate     Bitrate option of the opened channel, see BM_BitrateTypeDef for details.
 * @param[in]  rxfilter    CAN acceptance filters option of the opened channel, see BM_RxFilterTypeDef for details.
 * @param[in]  nrxfilters  Number of acceptance filters, usually there could be up to 2 filters.
 * @return Operation status, see BM_StatusTypeDef for details.
 */
BMAPI BM_StatusTypeDef BM_OpenEx(
        BM_ChannelHandle* handle,
        BM_ChannelInfoTypeDef* channelinfo,
        BM_CanModeTypeDef mode,
        BM_TerminalResistorTypeDef tres,
        const BM_BitrateTypeDef* bitrate,
        const BM_RxFilterTypeDef* rxfilter,
        int nrxfilters
        );

/**
 * @brief     Close an opened channel.
 * @param[in] handle  Handle to the channel to be closed.
 * @return    Operation status, see BM_StatusTypeDef for details.
 */
BMAPI BM_StatusTypeDef BM_Close(BM_ChannelHandle handle);

/**
 * @brief     Reset an opened channel.
 * @param[in] handle  Handle to the channel to be reset.
 * @return    Operation status, see BM_StatusTypeDef for details.
 * @note      The configuration options will not lost when the channel is reset, so BM_Reset() is basically identical to BM_Close() and then BM_OpenEx().
 */
BMAPI BM_StatusTypeDef BM_Reset(BM_ChannelHandle handle);

/**
 * @brief     Reset the device hardware which an opened channel belongs to, in case the device hardware is in unknown error state and is unrecoverable using BM_OpenEx.
 * @param[in] handle  Handle to a opened channel (which belongs to the physical device to reset).
 * @return    Operation status, see BM_StatusTypeDef for details.
 * @note      !!!CAUTION!!! Note this API will break all active USB/Ethernet connection, just like you have manually unplugged it and then plugged it back.
 *            All opened channel handles that belongs to the physical device under reset will be invalid after reset.
 *            You MUST re-open and re-configure all channels using BM_OpenEx or other configuration APIs after reset.
 */
BMAPI BM_StatusTypeDef BM_ResetDevice(BM_ChannelHandle handle);

/**
 * @brief     Activate an opened channel, and thus goes on bus for the selected port and channels. 
              At this point, the user can transmit and receive messages on the bus.
 * @param[in] handle  Handle to the channel to be activated.
 * @return    Operation status, see BM_StatusTypeDef for details.
 * @note      Channel is default to be activated after BM_OpenEx() is called.
 */
BMAPI BM_StatusTypeDef BM_Activate(BM_ChannelHandle handle);

/**
 * @brief     Deactivate an opened channel, and thus the selected channels goes off the bus and stay in BUSOFF state until re-activation.
 * @param[in] handle  Handle to the channel to be deactivated.
 * @return    Operation status, see BM_StatusTypeDef for details.
 * @note      Any call to BM_Write() or BM_Read() will return BM_ERROR_BUSOFF immediately if the channel is deactivated.
 */
BMAPI BM_StatusTypeDef BM_Deactivate(BM_ChannelHandle handle);

/**
 * @brief     Clear TX&RX message buffer of an opened channel.
 * @param[in] handle  Handle to the channel to be cleared.
 * @return    Operation status, see BM_StatusTypeDef for details.
 * @note      This function is available since BMAPI1.3, hardware status will not be changed when clearing buffer.
 */
BMAPI BM_StatusTypeDef BM_ClearBuffer(BM_ChannelHandle handle);

/**
 * @brief      Read any message/event out of the given channel.
 * @param[in]  handle  Handle to the channel to read from.
 * @param[out] data    A caller-allocated buffer to hold the message/event output, see BM_DataTypeDef for details.
 * @return     Operation status, see BM_StatusTypeDef for details.
 * @note       This function is non-blocked, and thus will return BM_ERROR_QRCVEMPTY if no message is received.
 *             Please use notifications to wait for Rx events and then read message/event out of BMAPI internal RX buffer, otherwise you could also poll the device periodically.
 */
BMAPI BM_StatusTypeDef BM_Read(BM_ChannelHandle handle, BM_DataTypeDef* data);

/**
 * @brief        Read multiple messages/events out of the given channel.
 * @param[in]    handle  Handle to the channel to read from.
 * @param[out]   data       A caller-allocated buffer to hold the messages/events array output, see BM_DataTypeDef for details.
 * @param[inout] nmessages  Number of read messages, user shall initialize this param with the size (in messages) of the data buffer.
 * @param[in]    timeout    Timeout (in milliseconds) before the message is received successfully from the bus.
 *                          Set any negative number (i.e. -1) to wait infinitely.
 *                          Set 0 if you would like to receive asynchronously: read from BMAPI internal buffer and return immediately, use BM_WaitForNotifications() before reading.
 * @return       Operation status, see BM_StatusTypeDef for details.
 * @note         This function is non-blocked, and thus will return BM_ERROR_QRCVEMPTY if not all messages are received.
 *               Please use notifications to wait for Rx events and then read message/event out of BMAPI internal RX buffer, otherwise you could also poll the device periodically.
 */
BMAPI BM_StatusTypeDef BM_ReadMultiple(BM_ChannelHandle handle, BM_DataTypeDef data[], uint32_t* nmessages, int timeout);

/**
 * @brief        Read data block using ISOTP protocol.
 *               This API enables rapid transmission using ISOTP without app intervention, a simple example would be reading VIN using UDS:
 *               uint8_t request[] = { 0x22, 0xF1, 0x80 };
 *               uint8_t response[4096];
 *               uint32_t nbytes = sizeof(response);
 *               BM_WriteIsotp(channel, request, sizeof(request), config);
 *               BM_ReadIsotp(channel, response, nbytes, config);
 *               assert(response[0] == 0x62 && response[1] == 0xF1 && response[2] == 0x80);
 * @param[in]    handle    Handle to the channel to read from.
 * @param[in]    data      A caller-allocated buffer to hold the data block output.
 * @param[inout] nbytes    Length of the received data block, in bytes. Caller must initialize this arg with the size of the caller-allocated buffer.
 * @param[in]    timeout   Timeout (in milliseconds) before the message is received successfully from the bus.
 *                         Set any negative number (i.e. -1) to wait infinitely.
 *                         Set 0 if you would like to receive asynchronously: read from BMAPI internal buffer and return immediately, use BM_WaitForNotifications() before reading.
 * @param[in]    config    ISOTP configuration used by current transfer.
 * @return     Operation status, see BM_StatusTypeDef for details.
 * @note       This function is allowed to be called from multiple threads since BMAPI1.5.
 */
BMAPI BM_StatusTypeDef BM_ReadIsotp(BM_ChannelHandle handle, const void* data, uint32_t* nbytes, int timeout, BM_IsotpConfigTypeDef* config);

/**
 * @brief      Read CAN message out of the given channel.
 * @param[in]  handle     Handle to the channel to read from.
 * @param[out] msg        A caller-allocated buffer to hold the CAN message output, see BM_CanMessageTypeDef for details.
 * @param[out] channel    The source channel ID from which the message is received, starting from zero, could be NULL if not required.
 * @param[out] timestamp  The device local high precision timestamp in microseconds, when the message is physically received on the CAN bus, could be NULL if not required.
 * @return     Operation status, see BM_StatusTypeDef for details. 
 * @note       Note this function is a simple wrapper of BM_Read(), see BM_Read() for details.
 */
BMAPI BM_StatusTypeDef BM_ReadCanMessage(BM_ChannelHandle handle, BM_CanMessageTypeDef* msg, uint32_t* channel, uint32_t* timestamp);

/**
 * @brief        Read multiple CAN messages out of the given channel.
 * @param[in]    handle  Handle to the channel to read from.
 * @param[out]   data       A caller-allocated buffer to hold the CAN message array output, see BM_CanMessageTypeDef for details.
 * @param[inout] nmessages  Number of read messages, user shall initialize this param with the size (in messages) of the data buffer.
 * @param[in]    timeout    Timeout (in milliseconds) before the message is received successfully from the bus.
 *                          Set any negative number (i.e. -1) to wait infinitely.
 *                          Set 0 if you would like to receive asynchronously: read from BMAPI internal buffer and return immediately, use BM_WaitForNotifications() before reading.
 * @param[out]   channel    The source channel ID from which the message is received, starting from zero, could be NULL if not required.
 * @param[out]   timestamp  The device local high precision timestamp array in microseconds, when the message is physically transmitted on the CAN bus, could be NULL if not required.
 * @return       Operation status, see BM_StatusTypeDef for details.
 * @note         This function is non-blocked, and thus will return BM_ERROR_QRCVEMPTY if not all messages are received.
 *               Please use notifications to wait for Rx events and then read message/event out of BMAPI internal RX buffer, otherwise you could also poll the device periodically.
 */
BMAPI BM_StatusTypeDef BM_ReadMultipleCanMessage(BM_ChannelHandle handle, BM_CanMessageTypeDef msg[], uint32_t* nmessages, int timeout, uint32_t channel[], uint32_t timestamp[]);

/**
 * @brief      Write any message/event to the given channel.
 * @param[in]  handle  Handle to the channel to write to.
 * @param[in]  data      A caller-allocated buffer to hold the message/event input, see BM_DataTypeDef for details.
 * @param[in]  timeout   Timeout (in milliseconds) before the message is transmitted successfully to the bus.
 *                       Set any negative number (i.e. -1) to wait infinitely.
 *                       Set 0 if you would like to transmit asynchronously: put to BMAPI internal buffer and return immediately, then receive TXCMPLT event over BM_Read() later.
 * @param[out] timestamp The device local high precision timestamp in microseconds, when the message is physically transmitted on the CAN bus, could be NULL if not required.
 * @return     Operation status, see BM_StatusTypeDef for details.
 * @note       This function is allowed to be called from multiple threads since BMAPI1.3.
 */
BMAPI BM_StatusTypeDef BM_Write(BM_ChannelHandle handle, const BM_DataTypeDef* data, int timeout, uint32_t* timestamp);

/**
 * @brief        Write multiple messages/events to the given channel.
 * @param[in]    handle    Handle to the channel to write to.
 * @param[in]    data      A caller-allocated buffer to hold the messages/events array input, see BM_DataTypeDef for details.
 * @param[inout] nmessages Number of written messages, user shall initialize this param with the size (in messages) of the data buffer.
 * @param[in]    timeout   Timeout (in milliseconds) before the message is transmitted successfully to the bus.
 *                         Set any negative number (i.e. -1) to wait infinitely.
 *                         Set 0 if you would like to transmit asynchronously: put to BMAPI internal buffer and return immediately, then receive TXCMPLT event over BM_Read() later.
 * @param[out]   timestamp The device local high precision timestamp array in microseconds, when the message is physically transmitted on the CAN bus, could be NULL if not required.
 * @return     Operation status, see BM_StatusTypeDef for details.
 * @note       This function is allowed to be called from multiple threads since BMAPI1.3.
 */
BMAPI BM_StatusTypeDef BM_WriteMultiple(BM_ChannelHandle handle, const BM_DataTypeDef data[], uint32_t* nmessages, int timeout, uint32_t timestamp[]);

/**
 * @brief        Write data block using ISOTP protocol.
 *               This API enables rapid transmission using ISOTP without app intervention, a simple example would be writing VIN using UDS:
 *               uint8_t request[] = { 0x2E, 0xF1, 0x80, ... ... };
 *               BM_WriteIsotp(channel, request, sizeof(request), config);
 * @param[in]    handle    Handle to the channel to write to.
 * @param[in]    data      A caller-allocated buffer to hold the data block input.
 * @param[in]    nbytes    Length of the data block, in bytes.
 * @param[in]    timeout   Timeout (in milliseconds) before any message segment is transmitted successfully to the bus.
 *                         Note this is only for bus level timeout waiting for CAN ACK, for setting ISOTP protocol timeouts, see BM_IsotpConfigTypeDef for details.
 * @param[in]    config    ISOTP configuration used by current transfer.
 * @return     Operation status, see BM_StatusTypeDef for details.
 * @note       This function is allowed to be called from multiple threads since BMAPI1.5.
 */
BMAPI BM_StatusTypeDef BM_WriteIsotp(BM_ChannelHandle handle, const void* data, uint32_t nbytes, int timeout, BM_IsotpConfigTypeDef* config);

/**
 * @brief      Write CAN message to the given channel.
 * @param[in]  handle     Handle to the channel to write to.
 * @param[in]  msg        A caller-allocated buffer to hold the CAN message output, see BM_CanMessageTypeDef for details.
 * @param[in]  _channel   The target channel ID to which the message is transmitted, starting from zero. This parameter is reserved for future, always 0 now.
 * @param[in]  timeout   Timeout (in milliseconds) before the message is transmitted successfully to the bus.
 *                       Set any negative number (i.e. -1) to wait infinitely.
 *                       Set 0 if you would like to transmit asynchronously: put to BMAPI internal buffer and return immediately, then receive TXCMPLT event over BM_Read() later.
 * @param[in]  timestamp The device local high precision timestamp in microseconds, when the message is physically transmitted on the CAN bus, could be NULL if not required.
 * @note       Note this function is a simple wrapper of BM_Write(), see BM_Write() for details.
 */
BMAPI BM_StatusTypeDef BM_WriteCanMessage(BM_ChannelHandle handle, BM_CanMessageTypeDef* msg, uint32_t _channel, int timeout, uint32_t* timestamp);

/**
 * @brief        Write multiple CAN messages to the given channel.
 * @param[in]    handle    Handle to the channel to write to.
 * @param[in]    msg       A caller-allocated buffer to hold the CAN message array input, see BM_CanMessageTypeDef for details.
 * @param[inout] nmessages Number of written messages, user shall initialize this param with the size (in messages) of the data buffer.
 * @param[in]    _channel  The target channel ID to which the message is transmitted, starting from zero. This parameter is reserved for future, always 0 now, or simply pass NULL into the API.
 * @param[in]    timeout   Timeout (in milliseconds) before the message is transmitted successfully to the bus.
 *                         Set any negative number (i.e. -1) to wait infinitely.
 *                         Set 0 if you would like to transmit asynchronously: put to BMAPI internal buffer and return immediately, then receive TXCMPLT event over BM_Read() later.
 * @param[out]   timestamp The device local high precision timestamp array in microseconds, when the message is physically transmitted on the CAN bus, could be NULL if not required.
 * @return     Operation status, see BM_StatusTypeDef for details.
 * @note       This function is allowed to be called from multiple threads since BMAPI1.3.
 */
BMAPI BM_StatusTypeDef BM_WriteMultipleCanMessage(BM_ChannelHandle handle, const BM_CanMessageTypeDef msg[], uint32_t* nmessages, uint32_t _channel[], int timeout, uint32_t timestamp[]);

/**
 * @brief Control the given channel, this is an advanced interface and is typically used internally by BMAPI.
 * @param[in]    handle   Handle to the channel to control.
 * @param[in]    command  Control command.
 * @param[in]    value    Control value.
 * @param[in]    index    Control index.
 * @param[inout] data     Control data, could be NULL.
 * @param[inout] nbytes   Length in bytes of the control data, could be zero.
 */
BMAPI BM_StatusTypeDef BM_Control(BM_ChannelHandle handle, uint8_t command, uint16_t value, uint16_t index, void* data, int nbytes);

/**
 * @brief      Get current CAN status of the given channel.
 * @param[in]  handle      Handle to the channel to operate on.
 * @param[out] statusinfo  Detailed information of current CAN status, see BM_CanStatusInfoTypedef for details.
 * @return     Current status code, see BM_StatusTypeDef for details.
 */
BMAPI BM_StatusTypeDef BM_GetStatus(BM_ChannelHandle handle, BM_CanStatusInfoTypedef* statusinfo);

/**
 * @brief      Get current local high precision device timestamp, in microseconds.
 * @param[in]  handle     Handle to the channel to operate on.
 * @param[out] timestamp  Timestamp value.
 * @return     Operation status, see BM_StatusTypeDef for details.
 */
BMAPI BM_StatusTypeDef BM_GetTimestamp(BM_ChannelHandle handle, uint32_t* timestamp);

/**
 * @brief      Get TX tasks option of the given channel.
 * @param[in]  handle    Handle to the channel to operate on.
 * @param[in]  txtasks   An array of TX task information, see BM_TxTaskTypeDef for details.
 * @param[in]  ntxtasks  Number of valid TX tasks in the txtasks array.
 * @return     Operation status, see BM_StatusTypeDef for details.
 */
BMAPI BM_StatusTypeDef BM_GetTxTasks(BM_ChannelHandle handle, BM_TxTaskTypeDef* txtasks, int ntxtasks);

/**
 * @brief      Get Message Routes option of the given channel.
 * @param[in]  handle    Handle to the channel to operate on.
 * @param[in]  msgroutes   An array of Message Routes information, see BM_MessageRouteTypeDef for details.
 * @param[in]  nmsgroute  Number of valid Message Routes in the routes array.
 * @return     Operation status, see BM_StatusTypeDef for details.
 */
BMAPI BM_StatusTypeDef BM_GetMsgRoutes(BM_ChannelHandle handle, BM_MessageRouteTypeDef* msgroutes, int nmsgroute);
/**
 * @brief      Set CAN mode option of the given channel.
 * @param[in]  handle  Handle to the channel to operate on.
 * @param[in]  mode    Expected CAN mode, see BM_CanModeTypeDef for details.
 * @return     Operation status, see BM_StatusTypeDef for details.
 */
BMAPI BM_StatusTypeDef BM_SetCanMode(BM_ChannelHandle handle, BM_CanModeTypeDef mode);

/**
 * @brief      Get CAN mode option of the given channel.
 * @param[in]  handle  Handle to the channel to operate on.
 * @param[in]  mode    Current CAN mode, see BM_CanModeTypeDef for details.
 * @return     Operation status, see BM_StatusTypeDef for details.
 */
BMAPI BM_StatusTypeDef BM_GetCanMode(BM_ChannelHandle handle, BM_CanModeTypeDef* mode);
/**
 * @brief      Set terminal resistor option of the given channel.
 * @param[in]  handle  Handle to the channel to operate on.
 * @param[in]  tres    Expected terminal resistor, see BM_TerminalResistorTypeDef for details.
 * @return     Operation status, see BM_StatusTypeDef for details.
 */
BMAPI BM_StatusTypeDef BM_SetTerminalRegister(BM_ChannelHandle handle, BM_TerminalResistorTypeDef  tres);

/**
 * @brief      Get terminal resistor option of the given channel.
 * @param[in]  handle  Handle to the channel to operate on.
 * @param[in]  tres    Current terminal resistor, see BM_TerminalResistorTypeDef for details.
 * @return     Operation status, see BM_StatusTypeDef for details.
 */
BMAPI BM_StatusTypeDef BM_GetTerminalRegister(BM_ChannelHandle handle, BM_TerminalResistorTypeDef* tres);

/**
 * @brief      Set bitrate option of the given channel.
 * @param[in]  handle  Handle to the channel to operate on.
 * @param[in]  bitrate Expected bitrate, see BM_BitrateTypeDef for details.
 * @return     Operation status, see BM_StatusTypeDef for details.
 */
BMAPI BM_StatusTypeDef BM_SetBitrate(BM_ChannelHandle handle, const BM_BitrateTypeDef* bitrate);

/**
 * @brief      Get bitrate option of the given channel.
 * @param[in]  handle  Handle to the channel to operate on.
 * @param[in]  bitrate Current bitrate, see BM_BitrateTypeDef for details.
 * @return     Operation status, see BM_StatusTypeDef for details.
 */
BMAPI BM_StatusTypeDef BM_GetBitrate(BM_ChannelHandle handle, BM_BitrateTypeDef* bitrate);

/**
 * @brief      Set TX tasks option of the given channel.
 * @param[in]  handle    Handle to the channel to operate on.
 * @param[in]  txtasks   An array of TX task information, see BM_TxTaskTypeDef for details.
 * @param[in]  ntxtasks  Number of valid TX tasks in the txtasks array.
 * @return     Operation status, see BM_StatusTypeDef for details.
 */
BMAPI BM_StatusTypeDef BM_SetTxTasks(BM_ChannelHandle handle, BM_TxTaskTypeDef* txtasks, int ntxtasks);

/**
 * @brief      Set Message Routes option of the given channel.
 * @param[in]  handle    Handle to the channel to operate on.
 * @param[in]  msgroutes   An array of Message Routes information, see BM_MessageRouteTypeDef for details.
 * @param[in]  nmsgroute  Number of valid Message Routes in the routes array.
 * @return     Operation status, see BM_StatusTypeDef for details.
 */
BMAPI BM_StatusTypeDef BM_SetMsgRoutes(BM_ChannelHandle handle, BM_MessageRouteTypeDef* msgroutes, int nmsgroute);

/**
 * @brief      Set RX filters option of the given channel.
 * @param[in]  handle      Handle to the channel to operate on.
 * @param[in]  rxfilters   An array of RX filter information, see BM_RxFilterTypeDef for details.
 * @param[in]  nrxfilters  Number of valid RX filters in the txtasks rxfilters.
 * @return     Operation status, see BM_StatusTypeDef for details.
 */
BMAPI BM_StatusTypeDef BM_SetRxFilters(BM_ChannelHandle handle, BM_RxFilterTypeDef* rxfilters, int nrxfilters);

/**
 * @brief      Get RX filters option of the given channel.
 * @param[in]  handle      Handle to the channel to operate on.
 * @param[in]  rxfilters   An array of RX filter information, see BM_RxFilterTypeDef for details.
 * @param[in]  nrxfilters  Number of valid RX filters in the txtasks rxfilters.
 * @return     Operation status, see BM_StatusTypeDef for details.
 */
BMAPI BM_StatusTypeDef BM_GetRxFilters(BM_ChannelHandle handle, BM_RxFilterTypeDef* rxfilters, int nrxfilters);

/**
 * @brief Get the platform/OS independent notification handle for the given channel, so that the application could wait for notifications later.
 * @param[in]  handle        Handle to the channel that owns the notification handle.
 * @param[out] notification  The platform/OS independent notification handle.
 * @return     Operation status, see BM_StatusTypeDef for details.
 * @note       By using notification handles in a background thread, it is easy to implement an asynchronous message receiver as below:
 * @code
 *             channel = BM_OpenCan(...);
 *             BM_GetNotification(channel, notification);
 *             while (!exit) {
 *               BM_WaitForNotifications(&notification, 1, -1); // Wait infinitely for new message notification.
 *               BM_ReadCanMessage(...);
 *             }
 * @endcode
 */
BMAPI BM_StatusTypeDef BM_GetNotification(BM_ChannelHandle handle, BM_NotificationHandle* notification);

/**
 * @brief     A platform/OS independent implementation to wait for single/multiple notification handles.
 * @param[in] handles     An array of channel notification handles.
 * @param[in] nhandles    Number of valid notification handles.
 * @param[in] ntimeoutms  This function will block current thread for ntimeoutms milliseconds if no notification is received.
 *                        Note this function will return immediately once a new notification is received, the ntimeoutms param is ignored in this normal case.
 * @return    This function returns the index in handles array of the channel from which a new notification is posted.
 */
BMAPI int BM_WaitForNotifications(BM_NotificationHandle handles[], int nhandles, int ntimeoutms);

/**
 * @brief      Translate error code to string, this is a helper function to ease application programming.
 * @param[in]  errorcode  The errorcode to be translated.
 * @param[out] buffer     A caller-allocated string buffer to hold the translated string.
 * @param[in]  nbytes     Number in bytes of the string buffer.
 * @param[in]  language   Reserved, only English is supported currently.
 */
BMAPI void BM_GetErrorText(BM_StatusTypeDef errorcode, char* buffer, int nbytes, uint16_t language);

/**
 * @brief      Translate data (i.e. CAN message) to string, this is a helper function to ease application programming.
 * @param[in]  data       The message data to be translated.
 * @param[out] buffer     A caller-allocated string buffer to hold the translated string.
 * @param[in]  nbytes     Number in bytes of the string buffer.
 * @param[in]  language   Reserved, only English is supported currently.
 */
BMAPI void BM_GetDataText(BM_DataTypeDef* data, char* buffer, int nbytes, uint16_t language);

/**
 * @brief      Get library log level.
 * @return     Current log level, all messages equal to or less than this level are currently printed on debug console.
 */
BMAPI BM_LogLevelTypeDef BM_GetLogLevel(void);

/**
 * @brief      Set library log level.
 * @param[in]  level       Target log level, all messages equal to or less than this level will be printed on debug console.
 */
BMAPI void BM_SetLogLevel(BM_LogLevelTypeDef level);

/**
 * @brief      Get library (*.dll|*.so) BMAPI version.
 * @return     32-bit version code:
 *             bit31-28 = major
 *             bit27-24 = minor
 *             bit23-16 = revision
 *             bit15-00 = build
 * @note       This API is used to get the library version, 
 *             in case that *.h is mismatch with *.dll|*.so, use the macro BM_API_VERSION defined in bmapi.h to check consistency.
 */
BMAPI uint32_t BM_GetVersion(void);
/**
 * @brief      Load Config option of the given channel.
 * @param[in]  handle    Handle to the channel to operate on.
 * @param[in]  configmask   Device configuration mask£¬ see BM_TableTypeTypeDef for details.
 * @return     Operation status, see BM_StatusTypeDef for details.
 */
BMAPI BM_StatusTypeDef BM_LoadConfig(BM_ChannelHandle handle, int configmask);
/**
 * @brief      Save Config option of the given channel.
 * @param[in]  handle    Handle to the channel to operate on.
 * @param[in]  configmask   Device configuration mask£¬ see BM_TableTypeTypeDef for details.
 * @return     Operation status, see BM_StatusTypeDef for details.
 */
BMAPI BM_StatusTypeDef BM_SaveConfig(BM_ChannelHandle handle, int configmask);
/**
 * @brief      Clear Config option of the given channel.
 * @param[in]  handle    Handle to the channel to operate on.
 * @param[in]  configmask   Device configuration mask£¬ see BM_TableTypeTypeDef for details.
 * @return     Operation status, see BM_StatusTypeDef for details.
 */
BMAPI BM_StatusTypeDef BM_ClearConfig(BM_ChannelHandle handle, int configmask);
#ifdef __cplusplus
};
#endif

#endif /* #ifndef __BMAPI_H__ */
/**
 * End of file
 */
