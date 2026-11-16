/*-----------------------------------------------------------------------------
 * EcSlaveInfo.cpp
 * Copyright                acontis technologies GmbH, Ravensburg, Germany
 * Response                 Holger Oelhaf
 * Description
 *---------------------------------------------------------------------------*/

/*-INCLUDES------------------------------------------------------------------*/
#include "EcDemoApp.h"

/*-FUNCTION-DEFINITIONS------------------------------------------------------*/

/***************************************************************************************************/
/**
 * \brief  Find a specific slaves offset (or print all slave information).
 *
 * \return EC_TRUE on success, EC_FALSE otherwise.
 */
EC_T_BOOL PrintSlaveInfos(T_EC_DEMO_APP_CONTEXT* pAppContext)
{
    EC_T_BOOL  bRetVal    = EC_FALSE;
    EC_T_DWORD dwRes      = EC_E_ERROR;
    EC_T_DWORD dwSlaveIdx = 0;

#if (defined INCLUDE_EC_MASTER) || (defined INCLUDE_EC_MONITOR)
    EC_T_DWORD dwNumConnectedSlaves = emGetNumConnectedSlaves(pAppContext->dwInstanceId);
#elif (defined INCLUDE_EC_SIMULATOR)
    EC_T_DWORD dwNumConnectedSlaves = esGetNumConnectedSlaves(pAppContext->dwInstanceId);
#endif

    EC_T_BUS_SLAVE_INFO oBusSlaveInfo;
    EC_T_CFG_SLAVE_INFO oCfgSlaveInfo;
    OsMemset(&oBusSlaveInfo, 0, sizeof(EC_T_BUS_SLAVE_INFO));
    OsMemset(&oCfgSlaveInfo, 0, sizeof(EC_T_CFG_SLAVE_INFO));

    /* get information about all bus slaves */
    for (dwSlaveIdx = 0; dwSlaveIdx < dwNumConnectedSlaves; dwSlaveIdx++)
    {
        EC_T_WORD wAutoIncAddress = (EC_T_WORD)(0 - dwSlaveIdx);

        /* get bus slave information */
#if (defined INCLUDE_EC_MASTER) || (defined INCLUDE_EC_MONITOR)
        dwRes = emGetBusSlaveInfo(pAppContext->dwInstanceId, EC_FALSE, wAutoIncAddress, &oBusSlaveInfo);
#elif (defined INCLUDE_EC_SIMULATOR)
        dwRes = esGetBusSlaveInfo(pAppContext->dwInstanceId, EC_FALSE, wAutoIncAddress, &oBusSlaveInfo);
#endif
        if (EC_E_NOERROR != dwRes)
        {
#if (defined INCLUDE_EC_MASTER) || (defined INCLUDE_EC_MONITOR)
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "PrintSlaveInfos(): emGetBusSlaveInfo: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
#elif (defined INCLUDE_EC_SIMULATOR)
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "PrintSlaveInfos(): esGetBusSlaveInfo: %s (0x%lx))\n", esGetText(0, dwRes), dwRes));
#endif
            bRetVal = EC_FALSE;
            goto Exit;
        }
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "******************************************************************************\n"));
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Slave ID............: 0x%08X\n", oBusSlaveInfo.dwSlaveId));
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Bus Index...........: %d\n", (EC_T_WORD)(0 - wAutoIncAddress)));
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Bus AutoInc Address.: 0x%04x (%4d)\n", oBusSlaveInfo.wAutoIncAddress, oBusSlaveInfo.wAutoIncAddress));
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Bus Station Address.: 0x%04x (%4d)\n", oBusSlaveInfo.wStationAddress, oBusSlaveInfo.wStationAddress));
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Bus Alias Address...: 0x%04x (%4d)\n", oBusSlaveInfo.wAliasAddress, oBusSlaveInfo.wAliasAddress));
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Vendor ID...........: 0x%08X = %s\n", oBusSlaveInfo.dwVendorId, SlaveVendorText((T_eEtherCAT_Vendor)oBusSlaveInfo.dwVendorId)));
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Product Code........: 0x%08X = %s\n", oBusSlaveInfo.dwProductCode, SlaveProdCodeText((T_eEtherCAT_Vendor)oBusSlaveInfo.dwVendorId, (T_eEtherCAT_ProductCode)oBusSlaveInfo.dwProductCode)));
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Revision............: 0x%08X   Serial Number: %d\n", oBusSlaveInfo.dwRevisionNumber, oBusSlaveInfo.dwSerialNumber));
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "ESC Type............: %s (0x%x)  Revision: %d  Build: %d\n", ESCTypeText(oBusSlaveInfo.byESCType), oBusSlaveInfo.byESCType, oBusSlaveInfo.byESCRevision, oBusSlaveInfo.wESCBuild));
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Connection at Port A: %s (to 0x%08X)\n", ((1 << 0) == (oBusSlaveInfo.wPortState & (1 << 0))) ? "yes" : "no", oBusSlaveInfo.adwPortSlaveIds[0]));
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Connection at Port D: %s (to 0x%08X)\n", ((1 << 3) == (oBusSlaveInfo.wPortState & (1 << 3))) ? "yes" : "no", oBusSlaveInfo.adwPortSlaveIds[3]));
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Connection at Port B: %s (to 0x%08X)\n", ((1 << 1) == (oBusSlaveInfo.wPortState & (1 << 1))) ? "yes" : "no", oBusSlaveInfo.adwPortSlaveIds[1]));
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Connection at Port C: %s (to 0x%08X)\n", ((1 << 2) == (oBusSlaveInfo.wPortState & (1 << 2))) ? "yes" : "no", oBusSlaveInfo.adwPortSlaveIds[2]));
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Line Crossed........: %s\n", (oBusSlaveInfo.bLineCrossed) ? "yes" : "no"));
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Line Crossed Flags..: 0x%x\n", oBusSlaveInfo.wLineCrossedFlags));

#if (defined NO_OS)
        ((CAtEmLogging*)GetLogParms()->pLogContext)->ProcessAllMsgs();
#endif

        /* get cfg slave information (matching bus slave) */
#if (defined INCLUDE_EC_MASTER) || (defined INCLUDE_EC_MONITOR)
        dwRes = emGetCfgSlaveInfo(pAppContext->dwInstanceId, EC_TRUE, oBusSlaveInfo.wStationAddress, &oCfgSlaveInfo);
#elif (defined INCLUDE_EC_SIMULATOR)
        dwRes = esGetCfgSlaveInfo(pAppContext->dwInstanceId, EC_TRUE, oBusSlaveInfo.wStationAddress, &oCfgSlaveInfo);
#endif
        if (EC_E_NOERROR != dwRes)
        {
#if (defined INCLUDE_EC_MASTER) || (defined INCLUDE_EC_MONITOR)
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "PrintSlaveInfos(): emGetCfgSlaveInfo: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
#elif (defined INCLUDE_EC_SIMULATOR)
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "PrintSlaveInfos(): esGetCfgSlaveInfo: %s (0x%lx))\n", esGetText(0, dwRes), dwRes));
#endif
            continue;
        }
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Cfg Station Address.: 0x%04x (%4d)\n", oCfgSlaveInfo.wStationAddress, oCfgSlaveInfo.wStationAddress));
        if (0 != oCfgSlaveInfo.dwPdSizeIn)
        {
            EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "PD IN    Byte.Bit offset: %d.%d   Size: %d bits (MSU %2d)\n", oCfgSlaveInfo.dwPdOffsIn / 8, oCfgSlaveInfo.dwPdOffsIn % 8, oCfgSlaveInfo.dwPdSizeIn, oCfgSlaveInfo.awMasterSyncUnitIn[0]));
        }
        if (0 != oCfgSlaveInfo.dwPdSizeOut)
        {
            EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "PD OUT   Byte.Bit offset: %d.%d   Size: %d bits (MSU %2d)\n", oCfgSlaveInfo.dwPdOffsOut / 8, oCfgSlaveInfo.dwPdOffsOut % 8, oCfgSlaveInfo.dwPdSizeOut, oCfgSlaveInfo.awMasterSyncUnitOut[0]));
        }

        if (0 != oCfgSlaveInfo.dwPdSizeIn2)
        {
            EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "PD IN  2 Byte.Bit offset: %d.%d   Size: %d bits (MSU %2d)\n", oCfgSlaveInfo.dwPdOffsIn2 / 8, oCfgSlaveInfo.dwPdOffsIn2 % 8, oCfgSlaveInfo.dwPdSizeIn2, oCfgSlaveInfo.awMasterSyncUnitIn[1]));
        }
        if (0 != oCfgSlaveInfo.dwPdSizeOut2)
        {
            EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "PD OUT 2 Byte.Bit offset: %d.%d   Size: %d bits (MSU %2d)\n", oCfgSlaveInfo.dwPdOffsOut2 / 8, oCfgSlaveInfo.dwPdOffsOut2 % 8, oCfgSlaveInfo.dwPdSizeOut2, oCfgSlaveInfo.awMasterSyncUnitOut[0]));
        }

        if (0 != oCfgSlaveInfo.dwPdSizeIn3)
        {
            EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "PD IN  3 Byte.Bit offset: %d.%d   Size: %d bits (MSU %2d)\n", oCfgSlaveInfo.dwPdOffsIn3 / 8, oCfgSlaveInfo.dwPdOffsIn3 % 8, oCfgSlaveInfo.dwPdSizeIn3, oCfgSlaveInfo.awMasterSyncUnitIn[2]));
        }
        if (0 != oCfgSlaveInfo.dwPdSizeOut3)
        {
            EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "PD OUT 3 Byte.Bit offset: %d.%d   Size: %d bits (MSU %2d)\n", oCfgSlaveInfo.dwPdOffsOut3 / 8, oCfgSlaveInfo.dwPdOffsOut3 % 8, oCfgSlaveInfo.dwPdSizeOut3, oCfgSlaveInfo.awMasterSyncUnitOut[0]));
        }
        if (0 != oCfgSlaveInfo.dwPdSizeIn4)
        {
            EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "PD IN  4 Byte.Bit offset: %d.%d   Size: %d bits (MSU %2d)\n", oCfgSlaveInfo.dwPdOffsIn4 / 8, oCfgSlaveInfo.dwPdOffsIn4 % 8, oCfgSlaveInfo.dwPdSizeIn4, oCfgSlaveInfo.awMasterSyncUnitIn[3]));
        }
        if (0 != oCfgSlaveInfo.dwPdSizeOut4)
        {
            EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "PD OUT 4 Byte.Bit offset: %d.%d   Size: %d bits (MSU %2d)\n", oCfgSlaveInfo.dwPdOffsOut4 / 8, oCfgSlaveInfo.dwPdOffsOut4 % 8, oCfgSlaveInfo.dwPdSizeOut4, oCfgSlaveInfo.awMasterSyncUnitOut[0]));
        }
#if (defined NO_OS)
        ((CAtEmLogging*)GetLogParms()->pLogContext)->ProcessAllMsgs();
#endif
    }
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "******************************************************************************\n"));

    bRetVal = EC_TRUE;
Exit:
    return bRetVal;
}

/***************************************************************************************************/
/**
 * \brief  Find a specific slaves offset (or print all slave information).
 *
 * \return EC_TRUE on success, EC_FALSE otherwise.
 */
EC_T_VOID PrintCfgSlavesInfo(T_EC_DEMO_APP_CONTEXT* pAppContext)
{
    EC_T_DWORD dwRes      = EC_E_ERROR;
    EC_T_DWORD dwSlaveIdx = 0;

#if (defined INCLUDE_EC_MASTER) || (defined INCLUDE_EC_MONITOR)
    EC_T_DWORD dwNumConfiguredSlaves = emGetNumConfiguredSlaves(pAppContext->dwInstanceId);
#elif (defined INCLUDE_EC_SIMULATOR)
    EC_T_DWORD dwNumConfiguredSlaves = esGetNumConfiguredSlaves(pAppContext->dwInstanceId);
#endif

    EC_T_CFG_SLAVE_INFO oCfgSlaveInfo;
    OsMemset(&oCfgSlaveInfo, 0, sizeof(EC_T_CFG_SLAVE_INFO));

    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "==================================================\n"));
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "=== Config slaves ================================\n"));
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "==================================================\n"));
    for (dwSlaveIdx = 0; dwSlaveIdx < dwNumConfiguredSlaves; dwSlaveIdx++)
    {
        EC_T_WORD wAutoIncAddress = (EC_T_WORD)(0 - dwSlaveIdx);

        /* get config slave information */
#if (defined INCLUDE_EC_MASTER) || (defined INCLUDE_EC_MONITOR)
        dwRes = emGetCfgSlaveInfo(pAppContext->dwInstanceId, EC_FALSE, wAutoIncAddress, &oCfgSlaveInfo);
#elif (defined INCLUDE_EC_SIMULATOR)
        dwRes = esGetCfgSlaveInfo(pAppContext->dwInstanceId, EC_FALSE, wAutoIncAddress, &oCfgSlaveInfo);
#endif
        if (EC_E_NOERROR != dwRes)
        {
#if (defined INCLUDE_EC_MASTER) || (defined INCLUDE_EC_MONITOR)
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "PrintCfgSlavesInfo(): emGetCfgSlaveInfo: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
#elif (defined INCLUDE_EC_SIMULATOR)
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "PrintCfgSlavesInfo(): esGetCfgSlaveInfo: %s (0x%lx))\n", esGetText(0, dwRes), dwRes));
#endif
            break;
        }
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "******************************************************************************\n"));
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Slave ID............: 0x%08X\n", oCfgSlaveInfo.dwSlaveId));
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Bus AutoInc Address.: 0x%04x (%4d)\n", oCfgSlaveInfo.wAutoIncAddress, oCfgSlaveInfo.wAutoIncAddress));
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Bus Station Address.: 0x%04x (%4d)\n", oCfgSlaveInfo.wStationAddress, oCfgSlaveInfo.wStationAddress));
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Vendor ID...........: 0x%08X = %s\n", oCfgSlaveInfo.dwVendorId, SlaveVendorText((T_eEtherCAT_Vendor)oCfgSlaveInfo.dwVendorId)));
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Product Code........: 0x%08X = %s\n", oCfgSlaveInfo.dwProductCode, SlaveProdCodeText((T_eEtherCAT_Vendor)oCfgSlaveInfo.dwVendorId, (T_eEtherCAT_ProductCode)oCfgSlaveInfo.dwProductCode)));
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Revision............: 0x%08X   Serial Number: %d\n", oCfgSlaveInfo.dwRevisionNumber, oCfgSlaveInfo.dwSerialNumber));
        if (0 != oCfgSlaveInfo.dwPdSizeIn)
        {
            EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "PD IN   Byte.Bit offset: %d.%d   Size: %d bits\n", oCfgSlaveInfo.dwPdOffsIn / 8, oCfgSlaveInfo.dwPdOffsIn % 8, oCfgSlaveInfo.dwPdSizeIn));
        }
        if (0 != oCfgSlaveInfo.dwPdSizeOut)
        {
            EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "PD OUT  Byte.Bit offset: %d.%d   Size: %d bits\n", oCfgSlaveInfo.dwPdOffsOut / 8, oCfgSlaveInfo.dwPdOffsOut % 8, oCfgSlaveInfo.dwPdSizeOut));
        }
        if (0 != oCfgSlaveInfo.dwPdSizeIn2)
        {
            EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "PD IN2  Byte.Bit offset: %d.%d   Size: %d bits\n", oCfgSlaveInfo.dwPdOffsIn2 / 8, oCfgSlaveInfo.dwPdOffsIn2 % 8, oCfgSlaveInfo.dwPdSizeIn2));
        }
        if (0 != oCfgSlaveInfo.dwPdSizeOut2)
        {
            EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "PD OUT2 Byte.Bit offset: %d.%d   Size: %d bits\n", oCfgSlaveInfo.dwPdOffsOut2 / 8, oCfgSlaveInfo.dwPdOffsOut2 % 8, oCfgSlaveInfo.dwPdSizeOut2));
        }
        if (0 != oCfgSlaveInfo.dwPdSizeIn3)
        {
            EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "PD IN3  Byte.Bit offset: %d.%d   Size: %d bits\n", oCfgSlaveInfo.dwPdOffsIn3 / 8, oCfgSlaveInfo.dwPdOffsIn3 % 8, oCfgSlaveInfo.dwPdSizeIn3));
        }
        if (0 != oCfgSlaveInfo.dwPdSizeOut3)
        {
            EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "PD OUT3 Byte.Bit offset: %d.%d   Size: %d bits\n", oCfgSlaveInfo.dwPdOffsOut3 / 8, oCfgSlaveInfo.dwPdOffsOut3 % 8, oCfgSlaveInfo.dwPdSizeOut3));
        }
        if (0 != oCfgSlaveInfo.dwPdSizeIn4)
        {
            EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "PD IN4  Byte.Bit offset: %d.%d   Size: %d bits\n", oCfgSlaveInfo.dwPdOffsIn4 / 8, oCfgSlaveInfo.dwPdOffsIn4 % 8, oCfgSlaveInfo.dwPdSizeIn4));
        }
        if (0 != oCfgSlaveInfo.dwPdSizeOut4)
        {
            EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "PD OUT4 Byte.Bit offset: %d.%d   Size: %d bits\n", oCfgSlaveInfo.dwPdOffsOut4 / 8, oCfgSlaveInfo.dwPdOffsOut4 % 8, oCfgSlaveInfo.dwPdSizeOut4));
        }
        if (0 != oCfgSlaveInfo.dwMbxSupportedProtocols)
        {
            EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Protocol supported: %s%s%s%s%s%s\n", (oCfgSlaveInfo.dwMbxSupportedProtocols & EC_MBX_PROTOCOL_AOE) ? "AoE " : "",
                (oCfgSlaveInfo.dwMbxSupportedProtocols & EC_MBX_PROTOCOL_EOE) ? "EoE " : "",
                (oCfgSlaveInfo.dwMbxSupportedProtocols & EC_MBX_PROTOCOL_COE) ? "CoE " : "",
                (oCfgSlaveInfo.dwMbxSupportedProtocols & EC_MBX_PROTOCOL_FOE) ? "FoE " : "",
                (oCfgSlaveInfo.dwMbxSupportedProtocols & EC_MBX_PROTOCOL_SOE) ? "SoE " : "",
                (oCfgSlaveInfo.dwMbxSupportedProtocols & EC_MBX_PROTOCOL_VOE) ? "VoE " : "\n"));
            EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Mailbox size   out: %06d / in: %06d\n", oCfgSlaveInfo.dwMbxOutSize, oCfgSlaveInfo.dwMbxInSize));
            if ((oCfgSlaveInfo.dwMbxOutSize2 != 0) || (oCfgSlaveInfo.dwMbxInSize2 != 0))
            {
                EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "BootMbx size   out: %06d / in: %06d\n", oCfgSlaveInfo.dwMbxOutSize2, oCfgSlaveInfo.dwMbxInSize2));
            }
        }
        else
        {
            EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Mailbox supported  : No\n"));
        }
#if (defined INCLUDE_HOTCONNECT)
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Slave is mandatory : %s\n", ((oCfgSlaveInfo.dwHCGroupIdx == 0) ? "Yes" : "No")));
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Slave is present   : %s\n", (oCfgSlaveInfo.bIsPresent ? "Yes" : "No")));
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "HC group is present: %s\n", (oCfgSlaveInfo.bIsPresent ? "Yes" : "No")));
#endif
#if (defined NO_OS)
        ((CAtEmLogging*)GetLogParms()->pLogContext)->ProcessAllMsgs();
#endif
    }
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "==================================================\n"));
    return;
}

/***************************************************************************************************/
/**
 * \brief  Print bus slaves information
 *
 * \return EC_TRUE on success, EC_FALSE otherwise.
 */
EC_T_VOID PrintBusSlavesInfo(T_EC_DEMO_APP_CONTEXT* pAppContext)
{
    EC_T_DWORD dwRes      = EC_E_ERROR;
    EC_T_DWORD dwSlaveIdx = 0;

#if (defined INCLUDE_EC_MASTER) || (defined INCLUDE_EC_MONITOR)
    EC_T_DWORD dwNumConnectedSlaves = emGetNumConnectedSlaves(pAppContext->dwInstanceId);
#elif (defined INCLUDE_EC_SIMULATOR)
    EC_T_DWORD dwNumConnectedSlaves = esGetNumConnectedSlaves(pAppContext->dwInstanceId);
#endif

    EC_T_BUS_SLAVE_INFO oBusSlaveInfo;
    OsMemset(&oBusSlaveInfo, 0, sizeof(EC_T_BUS_SLAVE_INFO));

    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "==================================================\n"));
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "=== Bus slaves ===================================\n"));
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "==================================================\n"));
    for (dwSlaveIdx = 0; dwSlaveIdx < dwNumConnectedSlaves; dwSlaveIdx++)
    {
        EC_T_WORD wAutoIncAddress = (EC_T_WORD)(0 - dwSlaveIdx);

        /* get bus slave information */
#if (defined INCLUDE_EC_MASTER) || (defined INCLUDE_EC_MONITOR)
        dwRes = emGetBusSlaveInfo(pAppContext->dwInstanceId, EC_FALSE, wAutoIncAddress, &oBusSlaveInfo);
#elif (defined INCLUDE_EC_SIMULATOR)
        dwRes = esGetBusSlaveInfo(pAppContext->dwInstanceId, EC_FALSE, wAutoIncAddress, &oBusSlaveInfo);
#endif
        if (EC_E_NOERROR != dwRes)
        {
#if (defined INCLUDE_EC_MASTER) || (defined INCLUDE_EC_MONITOR)
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "PrintBusSlavesInfo(): emGetBusSlaveInfo: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
#elif (defined INCLUDE_EC_SIMULATOR)
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "PrintBusSlavesInfo(): esGetBusSlaveInfo: %s (0x%lx))\n", esGetText(0, dwRes), dwRes));
#endif
            break;
        }
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "******************************************************************************\n"));
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Slave ID............: 0x%08X\n", oBusSlaveInfo.dwSlaveId));
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Bus Index...........: %d\n", (EC_T_WORD)(0 - wAutoIncAddress)));
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Bus AutoInc Address.: 0x%04x (%4d)\n", oBusSlaveInfo.wAutoIncAddress, oBusSlaveInfo.wAutoIncAddress));
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Bus Station Address.: 0x%04x (%4d)\n", oBusSlaveInfo.wStationAddress, oBusSlaveInfo.wStationAddress));
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Bus Alias Address...: 0x%04x (%4d)\n", oBusSlaveInfo.wAliasAddress, oBusSlaveInfo.wAliasAddress));
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Vendor ID...........: 0x%08X = %s\n", oBusSlaveInfo.dwVendorId, SlaveVendorText((T_eEtherCAT_Vendor)oBusSlaveInfo.dwVendorId)));
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Product Code........: 0x%08X = %s\n", oBusSlaveInfo.dwProductCode, SlaveProdCodeText((T_eEtherCAT_Vendor)oBusSlaveInfo.dwVendorId, (T_eEtherCAT_ProductCode)oBusSlaveInfo.dwProductCode)));
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Revision............: 0x%08X   Serial Number: %d\n", oBusSlaveInfo.dwRevisionNumber, oBusSlaveInfo.dwSerialNumber));
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "ESC Type............: %s (0x%x)  Revision: %d  Build: %d\n", ESCTypeText(oBusSlaveInfo.byESCType), oBusSlaveInfo.byESCType, oBusSlaveInfo.byESCRevision, oBusSlaveInfo.wESCBuild));
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Line Crossed........: %s\n", (oBusSlaveInfo.bLineCrossed) ? "yes" : "no \n"));
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Connection at Port A: %s (to 0x%08X)\n", ((1 << 0) == (oBusSlaveInfo.wPortState & (1 << 0))) ? "yes" : "no", oBusSlaveInfo.adwPortSlaveIds[0]));
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Connection at Port D: %s (to 0x%08X)\n", ((1 << 3) == (oBusSlaveInfo.wPortState & (1 << 3))) ? "yes" : "no", oBusSlaveInfo.adwPortSlaveIds[3]));
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Connection at Port B: %s (to 0x%08X)\n", ((1 << 1) == (oBusSlaveInfo.wPortState & (1 << 1))) ? "yes" : "no", oBusSlaveInfo.adwPortSlaveIds[1]));
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Connection at Port C: %s (to 0x%08X)\n", ((1 << 2) == (oBusSlaveInfo.wPortState & (1 << 2))) ? "yes" : "no", oBusSlaveInfo.adwPortSlaveIds[2]));
#if (defined NO_OS)
        ((CAtEmLogging*)GetLogParms()->pLogContext)->ProcessAllMsgs();
#endif
    };
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "==================================================\n"));
    return;
}

/***************************************************************************************************/
/**
 * \brief  Find a specific slave and return its fixed (ethercat) address
 *
 * \return EC_TRUE on success, EC_FALSE otherwise.
 */
EC_T_BOOL FindSlaveGetFixedAddr(
    T_EC_DEMO_APP_CONTEXT* pAppContext,
    EC_T_DWORD             dwSlaveInstance,       /**< [in]   Slave instance (0 = first matching, 1 = second, ...) */
    EC_T_DWORD             dwVendorId,            /**< [in]   Vendor Id of slave to search */
    EC_T_DWORD             dwProductCode,         /**< [in]   Product Code of slave to search */
    EC_T_WORD*             pwStationAddr          /**< [out]  Station address of slave */
)
{
    EC_T_DWORD dwRes              = EC_E_ERROR;
    EC_T_DWORD dwSlaveIdx         = 0;
    EC_T_DWORD dwSlaveInstanceCnt = 0;

#if (defined INCLUDE_EC_MASTER) || (defined INCLUDE_EC_MONITOR)
    EC_T_DWORD dwNumConnectedSlaves = emGetNumConnectedSlaves(pAppContext->dwInstanceId);
#elif (defined INCLUDE_EC_SIMULATOR)
    EC_T_DWORD dwNumConnectedSlaves = esGetNumConnectedSlaves(pAppContext->dwInstanceId);
#endif

    EC_T_BUS_SLAVE_INFO oBusSlaveInfo;
    OsMemset(&oBusSlaveInfo, 0, sizeof(EC_T_BUS_SLAVE_INFO));

    for (dwSlaveIdx = 0; dwSlaveIdx < dwNumConnectedSlaves; dwSlaveIdx++)
    {
        EC_T_WORD wAutoIncAddress = (EC_T_WORD)(0 - dwSlaveIdx);

        /* get information about bus slave */
#if (defined INCLUDE_EC_MASTER) || (defined INCLUDE_EC_MONITOR)
        dwRes = emGetBusSlaveInfo(pAppContext->dwInstanceId, EC_FALSE, wAutoIncAddress, &oBusSlaveInfo);
#elif (defined INCLUDE_EC_SIMULATOR)
        dwRes = esGetBusSlaveInfo(pAppContext->dwInstanceId, EC_FALSE, wAutoIncAddress, &oBusSlaveInfo);
#endif
        if (EC_E_NOERROR != dwRes)
        {
#if (defined INCLUDE_EC_MASTER) || (defined INCLUDE_EC_MONITOR)
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "FindSlaveGetFixedAddr(): emGetBusSlaveInfo: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
#elif (defined INCLUDE_EC_SIMULATOR)
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "FindSlaveGetFixedAddr(): esGetBusSlaveInfo: %s (0x%lx))\n", esGetText(0, dwRes), dwRes));
#endif
            continue;
        }
        if ((oBusSlaveInfo.dwVendorId == dwVendorId) && (oBusSlaveInfo.dwProductCode == dwProductCode))
        {
            if (dwSlaveInstanceCnt == dwSlaveInstance)
            {
                /* slave found */
                *pwStationAddr = oBusSlaveInfo.wStationAddress;
                return EC_TRUE;
            }
            dwSlaveInstanceCnt++;
        }
    }
    return EC_FALSE;
}

/***************************************************************************************************/
/**
 * \brief  Find a specific slave and return its fixed (ethercat) address
 *
 * \return EC_TRUE on success, EC_FALSE otherwise.
 */
EC_T_BOOL FindCfgSlaveGetFixedAddr(
    T_EC_DEMO_APP_CONTEXT* pAppContext,
    EC_T_DWORD             dwSlaveInstance,       /**< [in]   Slave instance (0 = first matching, 1 = second, ...) */
    EC_T_DWORD             dwVendorId,            /**< [in]   Vendor Id of slave to search */
    EC_T_DWORD             dwProductCode,         /**< [in]   Product Code of slave to search */
    EC_T_WORD*             pwStationAddr          /**< [out]  Station address of slave */
)
{
    EC_T_DWORD dwRes = EC_E_ERROR;
    EC_T_DWORD dwSlaveIdx = 0;
    EC_T_DWORD dwSlaveInstanceCnt = 0;

#if (defined INCLUDE_EC_MASTER) || (defined INCLUDE_EC_MONITOR)
    EC_T_DWORD dwNumConfiguredSlaves = emGetNumConfiguredSlaves(pAppContext->dwInstanceId);
#elif (defined INCLUDE_EC_SIMULATOR)
    EC_T_DWORD dwNumConfiguredSlaves = esGetNumConfiguredSlaves(pAppContext->dwInstanceId);
#endif

    EC_T_CFG_SLAVE_INFO oCfgSlaveInfo;
    OsMemset(&oCfgSlaveInfo, 0, sizeof(EC_T_CFG_SLAVE_INFO));

    for (dwSlaveIdx = 0; dwSlaveIdx < dwNumConfiguredSlaves; dwSlaveIdx++)
    {
        EC_T_WORD wAutoIncAddress = (EC_T_WORD)(0 - dwSlaveIdx);

        /* get information about bus slave */
#if (defined INCLUDE_EC_MASTER) || (defined INCLUDE_EC_MONITOR)
        dwRes = emGetCfgSlaveInfo(pAppContext->dwInstanceId, EC_FALSE, wAutoIncAddress, &oCfgSlaveInfo);
#elif (defined INCLUDE_EC_SIMULATOR)
        dwRes = esGetCfgSlaveInfo(pAppContext->dwInstanceId, EC_FALSE, wAutoIncAddress, &oCfgSlaveInfo);
#endif
        if (EC_E_NOERROR != dwRes)
        {
#if (defined INCLUDE_EC_MASTER) || (defined INCLUDE_EC_MONITOR)
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "FindCfgSlaveGetFixedAddr(): emGetCfgSlaveInfo: %s (0x%lx))\n", ecatGetText(dwRes), dwRes));
#elif (defined INCLUDE_EC_SIMULATOR)
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "FindCfgSlaveGetFixedAddr(): esGetCfgSlaveInfo: %s (0x%lx))\n", esGetText(0, dwRes), dwRes));
#endif
            continue;
        }
        if ((oCfgSlaveInfo.dwVendorId == dwVendorId) && (oCfgSlaveInfo.dwProductCode == dwProductCode))
        {
            if (dwSlaveInstanceCnt == dwSlaveInstance)
            {
                /* slave found */
                *pwStationAddr = oCfgSlaveInfo.wStationAddress;
                return EC_TRUE;
            }
            dwSlaveInstanceCnt++;
        }
    }
    return EC_FALSE;
}

#define VENDOR_TEXT(id, text)\
    case (id): pRet = (text);break;

/***************************************************************************************************/
/**
 * \brief  Slave Vendor Text.
 *
 * \return Vendor Text.
 */
const EC_T_CHAR* SlaveVendorText(T_eEtherCAT_Vendor eVendorId)
{
    const EC_T_CHAR* pRet = EC_NULL;

    switch (eVendorId)
    {
    VENDOR_TEXT(ecvendor_etg,                                "EtherCAT Technology Group");
    VENDOR_TEXT(ecvendor_beckhoff,                           "Beckhoff Automation GmbH");
#if (!defined EC_DEMO_TINY)
    VENDOR_TEXT(ecvendor_scuola_superiore_s_anna,            "Scuola Superiore S. Anna");
    VENDOR_TEXT(ecvendor_ixxat,                              "IXXAT Automation GmbH");
    VENDOR_TEXT(ecvendor_vector_informatik,                  "Vector Informatik GmbH");
    VENDOR_TEXT(ecvendor_knestel,                            "KNESTEL Technologie and Elektronik GmbH");
    VENDOR_TEXT(ecvendor_janz_tec,                           "Janz Tec AG");
    VENDOR_TEXT(ecvendor_a_and_c_shenyang_university,        "AandC Institute of Shenyang University of Technology");
    VENDOR_TEXT(ecvendor_cmz_sistemi,                        "CMZ Sistemi Elettronici");
    VENDOR_TEXT(ecvendor_jsl,                                "JSL Technology Co.,Ltd");
    VENDOR_TEXT(ecvendor_comemso,                            "comemso GmbH");
    VENDOR_TEXT(ecvendor_softing,                            "Softing Industrial Automation GmbH");
    VENDOR_TEXT(ecvendor_microcontrol,                       "MicroControl GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_pollmeier,                          "ESR Pollmeier GmbH");
    VENDOR_TEXT(ecvendor_beihang_university,                 "Beihang University, School of Mechanical Engineering and Automation");
    VENDOR_TEXT(ecvendor_gkg_precision_machine,              "GKG Precision Machine Co., Ltd.");
    VENDOR_TEXT(ecvendor_inatech,                            "Inatech Co., Ltd.");
    VENDOR_TEXT(ecvendor_kuebler,                            "Fritz Kuebler GmbH");
    VENDOR_TEXT(ecvendor_keb,                                "KEB Automation KG");
    VENDOR_TEXT(ecvendor_ajinextek,                          "AJINEXTEK Co. Ltd.");
    VENDOR_TEXT(ecvendor_lti,                                "LTI Motion GmbH");
    VENDOR_TEXT(ecvendor_esd_electronic_design,              "esd electronics gmbh");
    VENDOR_TEXT(ecvendor_m2i,                                "M2I Corporation");
    VENDOR_TEXT(ecvendor_nsd,                                "NSD Corporation");
    VENDOR_TEXT(ecvendor_shanghai_ecat_science,              "Shanghai ECAT Science and Technology Co.,Ltd");
    VENDOR_TEXT(ecvendor_hms_industrial_networks,            "HMS Industrial Networks AB");
    VENDOR_TEXT(ecvendor_epis_automation,                    "epis Automation GmbH & Co. KG");
    VENDOR_TEXT(ecvendor_shanghai_microport_medical,         "Shanghai MicroPort Medical (Group) Co., Ltd.");
    VENDOR_TEXT(ecvendor_festo,                              "Festo AG and Co. KG");
    VENDOR_TEXT(ecvendor_dst_robot,                          "DST Robot Co. Ltd.");
    VENDOR_TEXT(ecvendor_wago,                               "WAGO Kontakttechnik GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_wuhan_farley,                       "Wuhan Farley Laserlab Cutting Welding System Engineering Co., Ltd.");
    VENDOR_TEXT(ecvendor_acti,                               "ACTi Corporation");
    VENDOR_TEXT(ecvendor_boschrexroth,                       "Bosch Rexroth AG");
    VENDOR_TEXT(ecvendor_hongke,                             "Hongke Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_selcom_elettronica,                 "SELCOM ELETTRONICA s.p.a");
    VENDOR_TEXT(ecvendor_moog,                               "Moog GmbH");
    VENDOR_TEXT(ecvendor_intec_motion_systems,               "INTEC - Motion Systems GmbH");
    VENDOR_TEXT(ecvendor_highyag_lasertechnologie,           "HIGHYAG Lasertechnologie GmbH");
    VENDOR_TEXT(ecvendor_guangzhou_kossi,                    "Guangzhou Kossi Intelligent Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_port,                               "port GmbH");
    VENDOR_TEXT(ecvendor_buerkert_werke,                     "Buerkert Werke GmbH");
    VENDOR_TEXT(ecvendor_adtec_plasma,                       "Adtec Plasma Technology Co. Ltd.");
    VENDOR_TEXT(ecvendor_lenze,                              "Lenze Drive Systems GmbH (Lenze AG)");
    VENDOR_TEXT(ecvendor_shanghai_3crobot,                   "Shanghai 3cRobot Co.,Ltd.");
    VENDOR_TEXT(ecvendor_tianjin_fuyun_tianyi,               "Tianjin Fuyun Tianyi Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_mts_sensor,                         "MTS Sensor Technologie GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_tigris_electronic,                  "Tigris Electronic GmbH");
    VENDOR_TEXT(ecvendor_hilscher,                           "Hilscher GmbH");
    VENDOR_TEXT(ecvendor_industrial_software,                "Industrial Software Co.");
    VENDOR_TEXT(ecvendor_ever,                               "EVER s.n.c.");
    VENDOR_TEXT(ecvendor_murrelektronik,                     "Murrelektronik GmbH");
    VENDOR_TEXT(ecvendor_knorr_bremse,                       "Knorr-Bremse Powertech");
    VENDOR_TEXT(ecvendor_beijing_jingwei,                    "Beijing Jingwei New Technology Textile Machinery CO., LTD.");
    VENDOR_TEXT(ecvendor_komax,                              "Komax AG");
    VENDOR_TEXT(ecvendor_sew_eurodrive,                      "SEW-EURODRIVE GmbH & Co. (EETB)");
    VENDOR_TEXT(ecvendor_schleicher,                         "Schleicher Electronic Berlin GmbH");
    VENDOR_TEXT(ecvendor_incaa_computers,                    "INCAA Computers BV");
    VENDOR_TEXT(ecvendor_bachmann_electronic,                "Bachmann electronic GmbH");
    VENDOR_TEXT(ecvendor_rofin_sinar,                        "ROFIN-SINAR Laser GmbH");
    VENDOR_TEXT(ecvendor_fagor_automation,                   "Fagor Automation Sociedad Cooperativa");
    VENDOR_TEXT(ecvendor_kollmorgen,                         "Kollmorgen Corporation");
    VENDOR_TEXT(ecvendor_woodward_seg,                       "Woodward SEG GmbH & Co. KG");
    VENDOR_TEXT(ecvendor_bernecker_rainer_ie,                "Bernecker + Rainer Industrie-Elektronik Ges.m.b.H");
    VENDOR_TEXT(ecvendor_ina_oriental_motor,                 "INA ORIENTAL MOTOR CO., LTD.");
    VENDOR_TEXT(ecvendor_slc_sautter_lift,                   "SLC Sautter Lift Components GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_sibonac_laser,                      "SIBONAC Laser Technologies Co.,Ltd.");
    VENDOR_TEXT(ecvendor_tuev_sued_rail,                     "TueV SueD Rail GmbH");
    VENDOR_TEXT(ecvendor_infranor_electronics,               "Infranor SAS");
    VENDOR_TEXT(ecvendor_omron,                              "OMRON Corporation");
    VENDOR_TEXT(ecvendor_phoenix_contact,                    "Phoenix Contact GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_thinkvo,                            "THINKVO Automation Equipment Co.,Ltd.");
    VENDOR_TEXT(ecvendor_system,                             "System S.p.A.");
    VENDOR_TEXT(ecvendor_vacon_plc,                          "Vacon Plc");
    VENDOR_TEXT(ecvendor_gefran,                             "Gefran S.P.A.");
    VENDOR_TEXT(ecvendor_shenzhen,                           "SHENZHEN CO-TRUST TECHNOLOGY CO., LTD.");
    VENDOR_TEXT(ecvendor_elmo_motion,                        "Elmo Motion Control Ltd.");
    VENDOR_TEXT(ecvendor_hans_turck,                         "Hans Turck GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_konsept_elektronik,                 "Konsept Elektronik");
    VENDOR_TEXT(ecvendor_sontheim_industrie_elektronik,      "Sontheim Industrie Elektronik GmbH");
    VENDOR_TEXT(ecvendor_horiba_stec,                        "HORIBA STEC, Co., Ltd.");
    VENDOR_TEXT(ecvendor_hirschmann_automation,              "Hirschmann Automation and Control GmbH");
    VENDOR_TEXT(ecvendor_wieland,                            "Wieland Electric GmbH");
    VENDOR_TEXT(ecvendor_bejing_ae_precision_machinery,      "Bejing AandE Precision Machinery Co., Ltd.");
    VENDOR_TEXT(ecvendor_copley,                             "Copley Controls, a Division of Analogic Corporation");
    VENDOR_TEXT(ecvendor_pepperl_fuchs,                      "Pepperl+Fuchs GmbH");
    VENDOR_TEXT(ecvendor_johannes_huebner,                   "Johannes Huebner Fabrik elektrischer Maschinen GmbH");
    VENDOR_TEXT(ecvendor_bristol,                            "Bristol Industrial and Research Associates Ltd (Biral)");
    VENDOR_TEXT(ecvendor_jetter,                             "Jetter AG");
    VENDOR_TEXT(ecvendor_abb_oy_drives,                      "ABB Oy Drives");
    VENDOR_TEXT(ecvendor_stoeber,                            "STOEBER ANTRIEBSTECHNIK GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_aec,                                "AEC S.r.l.");
    VENDOR_TEXT(ecvendor_advanced_motion_controls,           "ADVANCED Motion Controls");
    VENDOR_TEXT(ecvendor_bloom_energy,                       "Bloom Energy (India) Private Limited");
    VENDOR_TEXT(ecvendor_comdel,                             "Comdel, Inc.");
    VENDOR_TEXT(ecvendor_densan,                             "DENSAN CO., LTD.");
    VENDOR_TEXT(ecvendor_messung_systems,                    "Mitsubishi Electric India Pvt. Ltd.");
    VENDOR_TEXT(ecvendor_bonfiglioli_vectron,                "Bonfiglioli Vectron MDS GmbH");
    VENDOR_TEXT(ecvendor_phase_motion_control,               "Phase Motion Control SpA");
    VENDOR_TEXT(ecvendor_diener_automation,                  "Diener Automation GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_plating_electronic,                 "plating electronic GmbH");
    VENDOR_TEXT(ecvendor_metronix,                           "Metronix Messgeraete und Elektronik GmbH");
    VENDOR_TEXT(ecvendor_ascon,                              "Ascon S.p.A.");
    VENDOR_TEXT(ecvendor_esab_atas,                          "ESAB-ATAS GmbH");
    VENDOR_TEXT(ecvendor_elektrobit_automotive,              "Elektrobit Automotive GmbH");
    VENDOR_TEXT(ecvendor_baumer_ivo,                         "Baumer IVO GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_ed_elettronica,                     "E.D. Elettronica Dedicata S.r.l.");
    VENDOR_TEXT(ecvendor_mclaren_group,                      "McLaren Group Limited");
    VENDOR_TEXT(ecvendor_guangdong_university,               "Guangdong University of Technology");
    VENDOR_TEXT(ecvendor_atos_spa,                           "Atos SpA");
    VENDOR_TEXT(ecvendor_giant_magellan_telescope,           "Giant Magellan Telescope Corporation");
    VENDOR_TEXT(ecvendor_controltechniques,                  "Control Techniques Ltd.");
    VENDOR_TEXT(ecvendor_maxon_motor,                        "maxon motor ag");
    VENDOR_TEXT(ecvendor_yacoub_automation,                  "Yacoub Automation GmbH");
    VENDOR_TEXT(ecvendor_precitec,                           "Precitec GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_south_china_university,             "South China University of Technology");
    VENDOR_TEXT(ecvendor_easydur,                            "Easydur Italiana di Renato Affri");
    VENDOR_TEXT(ecvendor_isac,                               "ISAC Srl.");
    VENDOR_TEXT(ecvendor_lmd,                                "LMD GmbH and Co. KG aA");
    VENDOR_TEXT(ecvendor_keba,                               "KEBA AG (Product Development - Control)");
    VENDOR_TEXT(ecvendor_wittenstein,                        "WITTENSTEIN motion control GmbH");
    VENDOR_TEXT(ecvendor_twk_elektronik,                     "TWK-Elektronik GmbH");
    VENDOR_TEXT(ecvendor_wittenstein_motion,                 "WITTENSTEIN motion control GmbH");
    VENDOR_TEXT(ecvendor_psa_elettronica,                    "PSA Elettronica di F. Grifa");
    VENDOR_TEXT(ecvendor_heitec,                             "HEITEC AG");
    VENDOR_TEXT(ecvendor_smc,                                "SMC Corporation");
    VENDOR_TEXT(ecvendor_eckelmann,                          "Eckelmann AG");
    VENDOR_TEXT(ecvendor_jvl_industri_elektronik,            "JVL Industri Elektronik A/S");
    VENDOR_TEXT(ecvendor_atesteo,                            "ATESTEO GmbH");
    VENDOR_TEXT(ecvendor_hottinger_baldwin,                  "Hottinger Baldwin Messtechnik GmbH");
    VENDOR_TEXT(ecvendor_leuze_electronic,                   "Leuze electronic GmbH + Co. KG");
    VENDOR_TEXT(ecvendor_weg,                                "WEG Equipamentos Eletricos S.A.");
    VENDOR_TEXT(ecvendor_jumo,                               "JUMO GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_hsd,                                "HSD S.p.A");
    VENDOR_TEXT(ecvendor_digital_electronics,                "Digital Electronics Corporation");
    VENDOR_TEXT(ecvendor_lika_electronic,                    "Lika Electronic Srl");
    VENDOR_TEXT(ecvendor_csm,                                "CSM GmbH");
    VENDOR_TEXT(ecvendor_duometric,                          "DUOmetric AG");
    VENDOR_TEXT(ecvendor_fenwal_controls,                    "Fenwal Controls of Japan,Ltd.");
    VENDOR_TEXT(ecvendor_scaime,                             "SCAIME S.A.S.");
    VENDOR_TEXT(ecvendor_tecnologix,                         "TECNOLOGIX Srl");
    VENDOR_TEXT(ecvendor_lpkf_motion_control,                "LPKF SolarQuipment GmbH");
    VENDOR_TEXT(ecvendor_fritz_faulhaber,                    "Dr. Fritz Faulhaber GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_fraunhofer_institut,                "Fraunhofer-Institut fuer Produktionsanlagen und Konstruktionstechnik IPK");
    VENDOR_TEXT(ecvendor_imc_messysteme,                     "imc Messysteme GmbH");
    VENDOR_TEXT(ecvendor_tmg,                                "TMG Technologie und Engineering GmbH");
    VENDOR_TEXT(ecvendor_ferrocontrol,                       "Ferrocontrol Steuerungssysteme GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_bluechips,                          "Bluechips Microhouse Co., Ltd.");
    VENDOR_TEXT(ecvendor_baumueller,                         "Baumueller Nuernberg GmbH");
    VENDOR_TEXT(ecvendor_engel,                              "ENGEL Elektroantriebe GmbH");
    VENDOR_TEXT(ecvendor_seltek,                             "Seltek Ltd.");
    VENDOR_TEXT(ecvendor_aerolas,                            "AeroLas GmbH");
    VENDOR_TEXT(ecvendor_metso,                              "Metso Automation Oy");
    VENDOR_TEXT(ecvendor_amp_and_moons,                      "Shanghai AMPandMOONS' Automation Co., Ltd.");
    VENDOR_TEXT(ecvendor_comsoft,                            "COMSOFT GmbH");
    VENDOR_TEXT(ecvendor_prima,                              "Prima Electro S.p.A.");
    VENDOR_TEXT(ecvendor_fraunhofer_ipk,                     "Fraunhofer-Institut fuer Produktionsanlagen und Konstruktionstechnik IPK");
    VENDOR_TEXT(ecvendor_mit_university,                     "Massachusetts Institute of Technology (MIT)");
    VENDOR_TEXT(ecvendor_foshan_korter,                      "Foshan Korter Automatic Precision Measurement and Control Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_pneumax,                            "Pneumax S.p.A.");
    VENDOR_TEXT(ecvendor_rta,                                "R.T.A. S.r.l.");
    VENDOR_TEXT(ecvendor_fev,                                "FEV GmbH");
    VENDOR_TEXT(ecvendor_bei_sensors,                        "BEI Sensors SAS");
    VENDOR_TEXT(ecvendor_zhejiang_universit,                 "Zhejiang University of Technology, College of Information Engineering");
    VENDOR_TEXT(ecvendor_asa_rt,                             "ASA-RT srl");
    VENDOR_TEXT(ecvendor_promess,                            "PROMESS Gesellschaft fuer Montage- und Pruefsysteme mbH");
    VENDOR_TEXT(ecvendor_promess_incorporated,               "Promess Incorporated");
    VENDOR_TEXT(ecvendor_matsusada,                          "Matsusada Precision Inc.");
    VENDOR_TEXT(ecvendor_leine_linde,                        "Leine and Linde AB");
    VENDOR_TEXT(ecvendor_siko,                               "SIKO GmbH");
    VENDOR_TEXT(ecvendor_deutschmann,                        "Deutschmann Automation GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_golden,                             "Golden A/S");
    VENDOR_TEXT(ecvendor_brunner_elektronik,                 "Brunner Elektronik AG");
    VENDOR_TEXT(ecvendor_heckner,                            "Heckner Electronics GmbH");
    VENDOR_TEXT(ecvendor_technosoft,                         "TECHNOSOFT S.A.");
    VENDOR_TEXT(ecvendor_kongsberg,                          "Kongsberg Maritime AS");
    VENDOR_TEXT(ecvendor_reo,                                "REO AG");
    VENDOR_TEXT(ecvendor_abb,                                "ABB AB, Jokab Safety");
    VENDOR_TEXT(ecvendor_aventics,                           "AVENTICS GmbH");
    VENDOR_TEXT(ecvendor_peyer_engineering,                  "Peyer Engineering");
    VENDOR_TEXT(ecvendor_robox,                              "Robox S.P.A.");
    VENDOR_TEXT(ecvendor_pmb,                                "PMB Elektronik GmbH");
    VENDOR_TEXT(ecvendor_sanyo_denki,                        "Sanyo Denki Co., Ltd.");
    VENDOR_TEXT(ecvendor_eurotherm,                          "Eurotherm Limited");
    VENDOR_TEXT(ecvendor_kobe_steel,                         "Kobe Steel, Ltd.");
    VENDOR_TEXT(ecvendor_regatron,                           "Regatron AG");
    VENDOR_TEXT(ecvendor_eaton,                              "Eaton Industries GmbH");
    VENDOR_TEXT(ecvendor_delta_electronics,                  "Delta Electronics, Inc.");
    VENDOR_TEXT(ecvendor_xeikon,                             "Xeikon N.V. - Xeikon Manufacturing and RandD Center");
    VENDOR_TEXT(ecvendor_numatics,                           "Numatics Inc.");
    VENDOR_TEXT(ecvendor_amk,                                "AMK Arnold Mueller GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_robatech,                           "Robatech AG");
    VENDOR_TEXT(ecvendor_national_instruments,               "National Instruments Corporation");
    VENDOR_TEXT(ecvendor_fernsteuergeraete_kurt_oelsch,      "Fernsteuergeraete Kurt Oelsch GmbH");
    VENDOR_TEXT(ecvendor_idam,                               "INA - Drives and Mechatronics GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_prueftechnik_ndt,                   "PRueFTECHNIK NDT GmbH");
    VENDOR_TEXT(ecvendor_zhejiang_qixing,                    "Zhejiang Qixing Electron Co., Ltd.");
    VENDOR_TEXT(ecvendor_tde_macno,                          "TDE MACNO S.p.A.");
    VENDOR_TEXT(ecvendor_esitron_electronic,                 "esitron-electronic GmbH");
    VENDOR_TEXT(ecvendor_itoh_denki,                         "ITOH DENKI CO.,LTD.");
    VENDOR_TEXT(ecvendor_real_time,                          "Real Time Automation, Inc.");
    VENDOR_TEXT(ecvendor_wachendorff,                        "Wachendorff Automation GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_systeme_helmholz,                   "Helmholz GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_pantec,                             "Pantec Engineering AG");
    VENDOR_TEXT(ecvendor_vipa,                               "VIPA Gesellschaft fuer Visualisierung und Prozessautomatisierung mbH");
    VENDOR_TEXT(ecvendor_weidmueller,                        "Weidmueller Interface GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_xian_jiaotong_university,           "Guangdong Xi'an Jiaotong University Academy");
    VENDOR_TEXT(ecvendor_abb_stotz_kontakt,                  "ABB Automation Products GmbH");
    VENDOR_TEXT(ecvendor_berghof_automationstechnik,         "Berghof Automation GmbH");
    VENDOR_TEXT(ecvendor_ns_system,                          "NS System Co., Ltd.");
    VENDOR_TEXT(ecvendor_wiedemann,                          "Sensor-Technik Wiedemann GmbH");
    VENDOR_TEXT(ecvendor_spezialantriebstechnik,             "Spezialantriebstechnik GmbH");
    VENDOR_TEXT(ecvendor_harmonic_drive_ll,                  "Harmonic Drive LLC");
    VENDOR_TEXT(ecvendor_stotz_feinmesstechnik,              "Stotz Feinmesstechnik GmbH");
    VENDOR_TEXT(ecvendor_dunkermotoren,                      "Dunkermotoren GmbH");
    VENDOR_TEXT(ecvendor_chengdu_crp,                        "Chengdu CRP Automation Control Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_fuji,                               "Fuji Electric Co., Ltd.");
    VENDOR_TEXT(ecvendor_trumpf_huettinger,                  "TRUMPF Huettinger GmbH + Co. KG");
    VENDOR_TEXT(ecvendor_aros,                               "Aros Electronics AB");
    VENDOR_TEXT(ecvendor_nanotec_electronic,                 "Nanotec Electronic GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_me_messsysteme,                     "ME-Messsysteme GmbH");
    VENDOR_TEXT(ecvendor_interroll_holding,                  "Interroll Holding GmbH");
    VENDOR_TEXT(ecvendor_ish,                                "ISH Ingenieursozietaet GmbH");
    VENDOR_TEXT(ecvendor_mkprecision,                        "MKPRECISION");
    VENDOR_TEXT(ecvendor_roche_diagnostics,                  "Roche Diagnostics AG");
    VENDOR_TEXT(ecvendor_toshiba_schneider,                  "Toshiba Schneider Inverter Corporation");
    VENDOR_TEXT(ecvendor_bihl_wiedemann,                     "Bihl-Wiedemann GmbH");
    VENDOR_TEXT(ecvendor_trinamic_motion_control,            "TRINAMIC Motion Control GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_hdt,                                "HDT Srl");
    VENDOR_TEXT(ecvendor_horner,                             "Horner APG LLC");
    VENDOR_TEXT(ecvendor_performance_motion_devices,         "Performance Motion Devices, Inc.");
    VENDOR_TEXT(ecvendor_univer,                             "UNIVER S.p.A.");
    VENDOR_TEXT(ecvendor_gerhartl,                           "C.L.GERHARTL Smart Systems GmbH");
    VENDOR_TEXT(ecvendor_ingenia_cat,                        "INGENIA-CAT, S.L.");
    VENDOR_TEXT(ecvendor_crevis,                             "CREVIS Co., Ltd.");
    VENDOR_TEXT(ecvendor_nimak,                              "NIMAK GmbH");
    VENDOR_TEXT(ecvendor_elap,                               "ELAP S.R.L.");
    VENDOR_TEXT(ecvendor_advanced_energy,                    "Advanced Energy Industries, Inc.");
    VENDOR_TEXT(ecvendor_pba_systems,                        "PBA Systems Pte Ltd");
    VENDOR_TEXT(ecvendor_oriental_motor,                     "ORIENTAL MOTOR CO., LTD.");
    VENDOR_TEXT(ecvendor_glentek,                            "Glentek, Inc.");
    VENDOR_TEXT(ecvendor_fronius,                            "Fronius International GmbH");
    VENDOR_TEXT(ecvendor_thk,                                "THK Co., Ltd.");
    VENDOR_TEXT(ecvendor_samick,                             "SAMICK THK CO.,LTD.");
    VENDOR_TEXT(ecvendor_joint_peer_systec,                  "Joint Peer Systec Corp.");
    VENDOR_TEXT(ecvendor_halstrup_walcher,                   "halstrup-walcher GmbH");
    VENDOR_TEXT(ecvendor_walcher,                            "Walcher Messtechnik GmbH");
    VENDOR_TEXT(ecvendor_trio_motion,                        "Trio Motion Technology Ltd");
    VENDOR_TEXT(ecvendor_servotronix,                        "Servotronix Motion Control Ltd.");
    VENDOR_TEXT(ecvendor_analytica,                          "Analytica GmbH");
    VENDOR_TEXT(ecvendor_metal_work,                         "Metal Work S.p.A");
    VENDOR_TEXT(ecvendor_kotmi,                              "Korea Textile Machinery Research Institute (KOTMI)");
    VENDOR_TEXT(ecvendor_digitronic,                         "Digitronic Automationsanlagen GmbH");
    VENDOR_TEXT(ecvendor_dental_manufacturing,               "Dental Manufacturing Unit GmbH");
    VENDOR_TEXT(ecvendor_lam,                                "LAM Technologies S.a.S.");
    VENDOR_TEXT(ecvendor_iep,                                "IEP Ingenieurbuero fuer Echtzeitprogrammierung GmbH");
    VENDOR_TEXT(ecvendor_exceet,                             "exceet electronics AG");
    VENDOR_TEXT(ecvendor_kyung_motion,                       "A-KYUNG Motion Inc.");
    VENDOR_TEXT(ecvendor_pi_electronics,                     "PI Electronics (H.K.) Ltd.");
    VENDOR_TEXT(ecvendor_toflo,                              "TOFLO CORPORATION");
    VENDOR_TEXT(ecvendor_axis,                               "AXIS CORPORATION");
    VENDOR_TEXT(ecvendor_emtas,                              "emtas GmbH");
    VENDOR_TEXT(ecvendor_codesys,                            "CODESYS GmbH");
    VENDOR_TEXT(ecvendor_sibotech,                           "SiboTech Automation Co., Ltd.");
    VENDOR_TEXT(ecvendor_ge_intelligent,                     "GE Intelligent Platforms");
    VENDOR_TEXT(ecvendor_shanghai_sany,                      "SHANGHAI SANY SCIENCE and TECHNOLOGY CO., LTD");
    VENDOR_TEXT(ecvendor_cybelec,                            "CYBELEC S.A.");
    VENDOR_TEXT(ecvendor_kostal,                             "KOSTAL Industrie Elektrik GmbH");
    VENDOR_TEXT(ecvendor_rs_automation,                      "RS Automation Co., Ltd.");
    VENDOR_TEXT(ecvendor_kc_tech,                            "K.C.Tech CO.,LTD.");
    VENDOR_TEXT(ecvendor_exlar,                              "Exlar Corporation");
    VENDOR_TEXT(ecvendor_draegerwerk,                        "Draegerwerk AG and Co. KGaA");
    VENDOR_TEXT(ecvendor_schaefer,                           "Schaefer Elektronik GmbH");
    VENDOR_TEXT(ecvendor_infineon_technologies,              "Infineon Technologies AG");
    VENDOR_TEXT(ecvendor_novatech,                           "Novatech-Group Ltd.");
    VENDOR_TEXT(ecvendor_tianjin_geneuo,                     "Tianjin Geneuo Technology Co.,Ltd.");
    VENDOR_TEXT(ecvendor_taian,                              "TAIAN TECHNOLOGY(WUXI)CO.,LTD.");
    VENDOR_TEXT(ecvendor_xmos,                               "XMOS Semiconductor");
    VENDOR_TEXT(ecvendor_mks,                                "MKS Denmark ApS");
    VENDOR_TEXT(ecvendor_promicon,                           "Promicon Elektronik GmbH + Co. KG");
    VENDOR_TEXT(ecvendor_hein_lanz,                          "Hein Lanz GmbH");
    VENDOR_TEXT(ecvendor_dieentwickler,                      "dieEntwickler Elektronik GmbH");
    VENDOR_TEXT(ecvendor_larsys,                             "LARsys-Automation GmbH");
    VENDOR_TEXT(ecvendor_procon,                             "Procon Electronics Pty Ltd");
    VENDOR_TEXT(ecvendor_hanyang,                            "HanYang System");
    VENDOR_TEXT(ecvendor_j_schneider,                        "J. Schneider Elektrotechnik GmbH");
    VENDOR_TEXT(ecvendor_motovario,                          "Motovario S.p.A.");
    VENDOR_TEXT(ecvendor_baldor_uk,                          "Baldor UK Ltd");
    VENDOR_TEXT(ecvendor_lite_on,                            "Lite-On Technology Corporation");
    VENDOR_TEXT(ecvendor_chieftek,                           "Chieftek Precision Co., Ltd.");
    VENDOR_TEXT(ecvendor_fenac,                              "Fenac Muehendislik San. ve Tic. Ltd. sti.");
    VENDOR_TEXT(ecvendor_applied_motion,                     "Applied Motion Products, Inc.");
    VENDOR_TEXT(ecvendor_shenzhen_xtec,                      "Shenzhen X-TEC Technology Co., Ltd");
    VENDOR_TEXT(ecvendor_shenzhen_zmotion,                   "Shenzhen Zmotion Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_pivotal_systems,                    "Pivotal Systems Corporation");
    VENDOR_TEXT(ecvendor_randy_nuernberger,                  "Randy Nuernberger Software und Mikroelektronik");
    VENDOR_TEXT(ecvendor_noda_radio,                         "Noda Radio Frequency Technologies Co., Ltd.");
    VENDOR_TEXT(ecvendor_microchip,                          "Microchip Technology Inc.");
    VENDOR_TEXT(ecvendor_ketels,                             "Speciaal Machinefabriek Ketels v.o.f.");
    VENDOR_TEXT(ecvendor_beck_ipc,                           "Beck IPC GmbH");
    VENDOR_TEXT(ecvendor_etas,                               "ETAS GmbH");
    VENDOR_TEXT(ecvendor_phytec_messtechnik,                 "PHYTEC Messtechnik GmbH");
    VENDOR_TEXT(ecvendor_anca_motion,                        "ANCA Motion Pty. Ltd");
    VENDOR_TEXT(ecvendor_fh_koeln,                           "Fachhochschule Koeln");
    VENDOR_TEXT(ecvendor_ipg_automotive,                     "IPG Automotive GmbH");
    VENDOR_TEXT(ecvendor_nuvation_research,                  "Nuvation Research Corporation");
    VENDOR_TEXT(ecvendor_tr,                                 "TR-Electronic GmbH");
    VENDOR_TEXT(ecvendor_gantner,                            "Gantner Instruments GmbH");
    VENDOR_TEXT(ecvendor_mks_systems,                        "MKS Instruments");
    VENDOR_TEXT(ecvendor_abb_robotics,                       "ABB AB");
    VENDOR_TEXT(ecvendor_unitro_fleischmann,                 "Unitro-Fleischmann");
    VENDOR_TEXT(ecvendor_zub_machine_control,                "zub machine control AG");
    VENDOR_TEXT(ecvendor_dspace,                             "dSPACE GmbH");
    VENDOR_TEXT(ecvendor_samsung,                            "Samsung Heavy Industries");
    VENDOR_TEXT(ecvendor_bce,                                "BCE Elektronik GmbH");
    VENDOR_TEXT(ecvendor_jaeger_messtechnik,                 "Jaeger Computergesteuerte Messtechnik GmbH");
    VENDOR_TEXT(ecvendor_tetra,                              "TETRA Gesellschaft fuer Sensorik, Robotik und Automation mbH");
    VENDOR_TEXT(ecvendor_justek,                             "Justek Inc");
    VENDOR_TEXT(ecvendor_baumer_thalheim,                    "Baumer Thalheim GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_elin_ebg_traction,                  "Elin EBG Traction GmbH");
    VENDOR_TEXT(ecvendor_meka_robotics,                      "Meka Robotics");
    VENDOR_TEXT(ecvendor_altera_japan,                       "Altera Japan Ltd.");
    VENDOR_TEXT(ecvendor_ebv_elektronik,                     "EBV Elektronik GmbH and Co KG");
    VENDOR_TEXT(ecvendor_igh,                                "Ingenieurgemeinschaft IgH");
    VENDOR_TEXT(ecvendor_iav,                                "IAV GmbH");
    VENDOR_TEXT(ecvendor_hitachi,                            "Hitachi Industrial Equipment Systems");
    VENDOR_TEXT(ecvendor_tenasys,                            "TenAsys Corp.");
    VENDOR_TEXT(ecvendor_pondis,                             "PONDis AG");
    VENDOR_TEXT(ecvendor_moog_italiana,                      "Moog Italiana S.r.l.");
    VENDOR_TEXT(ecvendor_wallner_automation,                 "Wallner Automation");
    VENDOR_TEXT(ecvendor_avl_list,                           "AVL List GmbH");
    VENDOR_TEXT(ecvendor_ritter_elektronik,                  "RITTER-Elektronik GmbH");
    VENDOR_TEXT(ecvendor_zwick,                              "Zwick GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_dresdenelektronik,                  "dresden elektronik ingenieurtechnik gmbh");
    VENDOR_TEXT(ecvendor_tokyo_keiso,                        "Tokyo Keiso Co., Ltd.");
    VENDOR_TEXT(ecvendor_philips_healthcare,                 "Philips Healthcare (CT Division)");
    VENDOR_TEXT(ecvendor_chess,                              "Chess B.V.");
    VENDOR_TEXT(ecvendor_nct,                                "NCT kft");
    VENDOR_TEXT(ecvendor_anywire,                            "Anywire Corporation");
    VENDOR_TEXT(ecvendor_shadow_robot,                       "Shadow Robot Company Ltd.");
    VENDOR_TEXT(ecvendor_fecon,                              "FeCon GmbH");
    VENDOR_TEXT(ecvendor_fh_suedwestfahlen,                  "FH Suedwestfalen, Fachbereich Elektrische Energietechnik");
    VENDOR_TEXT(ecvendor_add2,                               "add2 Ldt");
    VENDOR_TEXT(ecvendor_arm_automation,                     "ARM Automation, Inc.");
    VENDOR_TEXT(ecvendor_knapp_logistik,                     "KNAPP AG");
    VENDOR_TEXT(ecvendor_getriebebau_nord,                   "Getriebebau NORD GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_yaskawa,                            "Yaskawa Electric Corporation");
    VENDOR_TEXT(ecvendor_oki,                                "OKI IDS Co., Ltd.");
    VENDOR_TEXT(ecvendor_takasaki_kyoudou,                   "Takasaki Kyoudou Computing Center Co.");
    VENDOR_TEXT(ecvendor_nittetsu_elex,                      "NITTETSU ELEX Co., Ltd.");
    VENDOR_TEXT(ecvendor_unjo,                               "Unjo AB");
    VENDOR_TEXT(ecvendor_eads_deutschland,                   "Airbus Defence and Space GmbH");
    VENDOR_TEXT(ecvendor_acs_motion_control,                 "ACS Motion Control Ltd.");
    VENDOR_TEXT(ecvendor_keyence,                            "KEYENCE Corporation");
    VENDOR_TEXT(ecvendor_mefi,                               "MEFI s.r.o.");
    VENDOR_TEXT(ecvendor_mut,                                "m-u-t AG Messgeraete fuer Medizin- und Umwelttechnik");
    VENDOR_TEXT(ecvendor_isw_uni_stuttgart,                  "Universitaet Stuttgart, Institut ISW");
    VENDOR_TEXT(ecvendor_elsena,                             "ELSENA, Inc.");
    VENDOR_TEXT(ecvendor_be_semiconductor,                   "BE Semiconductor Industries N.V.");
    VENDOR_TEXT(ecvendor_hauni_lni,                          "Hauni LNI Electronics S.A.");
    VENDOR_TEXT(ecvendor_etel,                               "ETEL S.A.");
    VENDOR_TEXT(ecvendor_vat_vakuumventile,                  "VAT Vakuumventile AG");
    VENDOR_TEXT(ecvendor_laytec,                             "LayTec AG");
    VENDOR_TEXT(ecvendor_num,                                "NUM AG");
    VENDOR_TEXT(ecvendor_hauni_maschinenbau,                 "Hauni Maschinenbau GmbH");
    VENDOR_TEXT(ecvendor_exatronic,                          "Exatronic, Engenharia ElectrAnica, Lda");
    VENDOR_TEXT(ecvendor_iim_chinese_aos,                    "Chinese Academy of Sciences, Institute of Intelligent Machines");
    VENDOR_TEXT(ecvendor_tu_eindhoven,                       "Eindhoven University of Technology");
    VENDOR_TEXT(ecvendor_scansonic,                          "Scansonic MI GmbH");
    VENDOR_TEXT(ecvendor_shanghai_sodick_sw,                 "Shanghai Sodick Software Co., Ltd.");
    VENDOR_TEXT(ecvendor_chuo_electronics,                   "CHUO ELECTRONICS CO., LTD");
    VENDOR_TEXT(ecvendor_agie,                               "Agie Charmilles SA");
    VENDOR_TEXT(ecvendor_hei_canton_de_vaud,                 "miControl GmbH");
    VENDOR_TEXT(ecvendor_jenny_science,                      "Jenny Science AG");
    VENDOR_TEXT(ecvendor_industrial_control_communications,  "Industrial Control Communications, Inc.");
    VENDOR_TEXT(ecvendor_ckd_elektrotechnika,                "ELEKTROTECHNIKA, a.s.");
    VENDOR_TEXT(ecvendor_qem,                                "QEM S.r.l.");
    VENDOR_TEXT(ecvendor_simatex,                            "Simatex AG");
    VENDOR_TEXT(ecvendor_kithara,                            "Kithara Software GmbH");
    VENDOR_TEXT(ecvendor_converteam,                         "GE Energy Power Conversion GmbH");
    VENDOR_TEXT(ecvendor_ara,                                "ARA apparatenfabriek b.v.");
    VENDOR_TEXT(ecvendor_tata_consultancy,                   "Tata Consultancy Services Ltd.");
    VENDOR_TEXT(ecvendor_harmonic_drive,                     "Harmonic Drive Systems Inc.");
    VENDOR_TEXT(ecvendor_tiab,                               "Tiab Limited");
    VENDOR_TEXT(ecvendor_rkc_instrument,                     "RKC INSTRUMENT INC.");
    VENDOR_TEXT(ecvendor_switched_reluctance,                "Switched Reluctance Drives Ltd.");
    VENDOR_TEXT(ecvendor_avnet_electronics,                  "Avnet Electronics Marketing");
    VENDOR_TEXT(ecvendor_abb_force_measurement,              "ABB AB");
    VENDOR_TEXT(ecvendor_yamaha_motor,                       "Yamaha Motor Co., Ltd.");
    VENDOR_TEXT(ecvendor_kunbus,                             "KUNBUS GmbH");
    VENDOR_TEXT(ecvendor_acd_antriebstechnik,                "ACD Antriebstechnik GmbH");
    VENDOR_TEXT(ecvendor_bronkhorst,                         "Bronkhorst High-Tech B.V.");
    VENDOR_TEXT(ecvendor_k_mecs,                             "K.MECS Co., Ltd.");
    VENDOR_TEXT(ecvendor_thomson_broadcast,                  "Ampegon AG");
    VENDOR_TEXT(ecvendor_ufg_elettronica,                    "UFG Elettronica s.r.l.");
    VENDOR_TEXT(ecvendor_xilinx,                             "Xilinx Inc.");
    VENDOR_TEXT(ecvendor_abb_power_systems,                  "ABB AB");
    VENDOR_TEXT(ecvendor_servoland,                          "Servoland Corporation");
    VENDOR_TEXT(ecvendor_hivertec,                           "Hivertec, Inc.");
    VENDOR_TEXT(ecvendor_fike_europe,                        "Fike Europe B.v.b.a.");
    VENDOR_TEXT(ecvendor_omicron,                            "OMICRON electronics GmbH");
    VENDOR_TEXT(ecvendor_fike,                               "Fike Europe B.v.b.a.");
    VENDOR_TEXT(ecvendor_ropex,                              "ROPEX Industrie-Elektronik GmbH");
    VENDOR_TEXT(ecvendor_tlu,                                "TLU - Thueringer Leistungselektronik Union GmbH");
    VENDOR_TEXT(ecvendor_prodrive,                           "Prodrive Technologies B.V.");
    VENDOR_TEXT(ecvendor_miho_inspektionssysteme,            "miho Inspektionssysteme GmbH");
    VENDOR_TEXT(ecvendor_tokyo_electron,                     "Tokyo Electron Device Limited");
    VENDOR_TEXT(ecvendor_lintec,                             "LINTEC CO., LTD.");
    VENDOR_TEXT(ecvendor_simplex_vision,                     "Symplex Vision Systems GmbH");
    VENDOR_TEXT(ecvendor_seiko_epson,                        "Seiko Epson Corporation");
    VENDOR_TEXT(ecvendor_zinser,                             "ZINSER GmbH");
    VENDOR_TEXT(ecvendor_abk_technology,                     "abk-technology GmbH");
    VENDOR_TEXT(ecvendor_sus,                                "SUS Corporation");
    VENDOR_TEXT(ecvendor_trsystems,                          "TRsystems GmbH");
    VENDOR_TEXT(ecvendor_harmonic_drive_ag,                  "Harmonic Drive AG");
    VENDOR_TEXT(ecvendor_staeubli_faverges,                  "Staeubli Faverges SCA");
    VENDOR_TEXT(ecvendor_scienlab_electronic,                "ScienLab electronic systems GmbH");
    VENDOR_TEXT(ecvendor_fujisoft,                           "FUJISOFT Incorporated");
    VENDOR_TEXT(ecvendor_iai_corporation,                    "IAI Corporation");
    VENDOR_TEXT(ecvendor_promavtomatika,                     "PromAvtomatika");
    VENDOR_TEXT(ecvendor_kistler_instrumente,                "Kistler Instrumente AG");
    VENDOR_TEXT(ecvendor_lauda_wobser,                       "LAUDA DR. R. WOBSER GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_schweitzer_engineering_labs,        "Schweitzer Engineering Laboratories, Inc.");
    VENDOR_TEXT(ecvendor_vital_systems,                      "Vital Systems Inc.");
    VENDOR_TEXT(ecvendor_mutracx,                            "Muehlbauer AG");
    VENDOR_TEXT(ecvendor_algo,                               "Algo System Co., Ltd.");
    VENDOR_TEXT(ecvendor_muehlbauer,                         "Muehlbauer AG");
    VENDOR_TEXT(ecvendor_deto_drive,                         "DETO drive systems GmbH");
    VENDOR_TEXT(ecvendor_sealevel_systems,                   "Sealevel Systems, Inc.");
    VENDOR_TEXT(ecvendor_igm_robotersysteme,                 "igm Robotersysteme AG");
    VENDOR_TEXT(ecvendor_wittenstein_electronics,            "WITTENSTEIN electronics GmbH");
    VENDOR_TEXT(ecvendor_zbe,                                "ZBE Inc.");
    VENDOR_TEXT(ecvendor_fraunhofer_iosb_ina,                "Fraunhofer IOSB-INA Kompetenzzentrum Industrial Automation");
    VENDOR_TEXT(ecvendor_skf_magnetic_bearings,              "SKF Magnetic Bearings");
    VENDOR_TEXT(ecevndor_galil_motion_control,               "Galil Motion Control Inc.");
    VENDOR_TEXT(ecvendor_ihi,                                "IHI Corporation");
    VENDOR_TEXT(ecvendor_wenglor_sensoric,                   "wenglor sensoric gmbh");
    VENDOR_TEXT(ecvendor_ingeteam,                           "Ingeteam Technology S.A.");
    VENDOR_TEXT(ecvendor_texas_instruments,                  "Texas Instruments Incorporated");
    VENDOR_TEXT(ecvendor_micro_vu,                           "Micro-Vu Corporation");
    VENDOR_TEXT(ecvendor_oehri_electronic,                   "oehri electronic ag");
    VENDOR_TEXT(ecvendor_triphase,                           "Triphase N.V.");
    VENDOR_TEXT(ecvendor_glass_soft,                         "Glass Soft - Robotica and Sistemas Lda");
    VENDOR_TEXT(ecvendor_cambridge_medical,                  "Cambridge Medical Robotics Limited");
    VENDOR_TEXT(ecvendor_china_machinery,                    "China Machinery International Engineering Design and Research Institute CO.,LTD.");
    VENDOR_TEXT(ecvendor_kastanienbaum,                      "Kastanienbaum GmbH");
    VENDOR_TEXT(ecvendor_hanyoung,                           "HANYOUNG NUX CO., LTD");
    VENDOR_TEXT(ecvendor_sle_quality,                        "SLE quality engineering GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_omicron_nanotechnology,             "Omicron NanoTechnology GmbH");
    VENDOR_TEXT(ecvendor_micromeritics,                      "Micromeritics Instrument Corporation");
    VENDOR_TEXT(ecvendor_trumpf,                             "TRUMPF Laser- und Systemtechnik GmbH");
    VENDOR_TEXT(ecvendor_beratron,                           "Beratron GmbH");
    VENDOR_TEXT(ecvendor_horiba,                             "HORIBA Europe GmbH");
    VENDOR_TEXT(ecvendor_heinz,                              "Heinz Siegfried AG");
    VENDOR_TEXT(ecvendor_cebora,                             "Cebora S.p.A.");
    VENDOR_TEXT(ecvendor_west,                               "W.E.ST Elektronik GmbH");
    VENDOR_TEXT(ecvendor_gomtec,                             "gomTec GmbH");
    VENDOR_TEXT(ecvendor_sieb_meyer,                         "SIEB and MEYER AG");
    VENDOR_TEXT(ecvendor_harbin,                             "Harbin Robotics Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_protechna_herbst,                   "Protechna Herbst GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_taeha,                              "TAEHA Mechatronics Co., Ltd.");
    VENDOR_TEXT(ecvendor_wittmann,                           "WITTMANN Kunststoffgeraete GmbH");
    VENDOR_TEXT(ecvendor_iotec,                              "iotec GmbH");
    VENDOR_TEXT(ecvendor_prodel,                             "Prodel Technologies");
    VENDOR_TEXT(ecvendor_leland_stanford_university,         "The Leland Stanford Junior University, Department of Bioengineering");
    VENDOR_TEXT(ecvendor_tarasheh,                           "Tarasheh System Pishro .co. Ltd");
    VENDOR_TEXT(ecvendor_cs_lab,                             "CS-Lab s.c. Janusz Wawak, Andrzej Rogozynski, Szymon Paprocki");
    VENDOR_TEXT(ecvendor_elitron,                            "Elitron IPM s.r.l.");
    VENDOR_TEXT(ecvendor_koryo,                              "KORYO ELECTRONICS CO.,LTD.");
    VENDOR_TEXT(ecvendor_shihlin,                            "Shihlin Electric and Engineering Corporation");
    VENDOR_TEXT(ecvendor_kookmin,                            "Kookmin University, Graduate School of Automotive Engineering");
    VENDOR_TEXT(ecvendor_techmation,                         "Techmation Co., Ltd.");
    VENDOR_TEXT(ecvendor_zapi,                               "ZAPI S.p.A");
    VENDOR_TEXT(ecvendor_claus_pribbernow,                   "Claus Pribbernow Mikrosystementwicklung eProcessorSolutions");
    VENDOR_TEXT(ecvendor_pragati,                            "Pragati Automation PVT. Limited");
    VENDOR_TEXT(ecvendor_siemens_software,                   "Siemens Industry Software B.V.");
    VENDOR_TEXT(ecvendor_micronova,                          "MicroNova AG");
    VENDOR_TEXT(ecvendor_xian_aerospace,                     "Xi'An Aerospace Precision Electromechanical Institute");
    VENDOR_TEXT(ecvendor_mergenthaler,                       "Dr. Mergenthaler GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_china_national_machinery,           "China National Machinery Industry Corporation");
    VENDOR_TEXT(ecvendor_berufliches_schulzentrum,           "Berufliches Schulzentrum Hof, Staatliche Fachschule fuer Technik");
    VENDOR_TEXT(ecvendor_ndr,                                "NDR Co., Ltd");
    VENDOR_TEXT(ecvendor_npk_msa,                            "NPK MSA LLC");
    VENDOR_TEXT(ecvendor_southeast_university,               "Southeast University, School of Mechanical Engineering");
    VENDOR_TEXT(ecvendor_shanghai_baosight,                  "Shanghai Baosight Software Co., Ltd.");
    VENDOR_TEXT(ecvendor_hakko,                              "Hakko Electronics Co., Ltd.");
    VENDOR_TEXT(ecvendor_gmk,                                "GMK electronic design GmbH");
    VENDOR_TEXT(ecvendor_simtec,                             "SIMTEC Elektronik GmbH");
    VENDOR_TEXT(ecvendor_teconcept,                          "TEConcept GmbH");
    VENDOR_TEXT(ecvendor_ess,                                "ESS Co., Ltd.");
    VENDOR_TEXT(ecvendor_mabi,                               "MABI AG - Robotic");
    VENDOR_TEXT(ecvendor_optoforce,                          "OptoForce Ltd.");
    VENDOR_TEXT(ecvendor_toshiba_mitsubishi,                 "TOSHIBA MITSUBISHI-ELECTRIC INDUSTRIAL SYSTEMS CORPORATION");
    VENDOR_TEXT(ecvendor_wittenstein_ternary,                "WITTENSTEIN ternary Co.,Ltd.");
    VENDOR_TEXT(ecvendor_shanghai_friendess,                 "Shanghai Friendess Electronic Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_aversan,                            "Aversan Inc.");
    VENDOR_TEXT(ecvendor_lorch,                              "Lorch Schweisstechnik GmbH");
    VENDOR_TEXT(ecvendor_lotes,                              "LOTES (GuangZhou) CO., LTD.");
    VENDOR_TEXT(ecvendor_sungkyunkwan_university,            "Sungkyunkwan University, School of Mechanical Engineering");
    VENDOR_TEXT(ecvendor_tsinghua_university,                "Tsinghua University, Graduate School at Shenzhen");
    VENDOR_TEXT(ecvendor_los_andes_university,               "Universidad de los Andes, Faculty of Engineering");
    VENDOR_TEXT(ecvendor_pfu,                                "PFU LIMITED");
    VENDOR_TEXT(ecvendor_slovak_university,                  "Slovak University of Technology in Bratislava, Faculty of Electrical Engineering and Information Technology");
    VENDOR_TEXT(ecvendor_esomatec,                           "Esomatec GmbH");
    VENDOR_TEXT(ecvendor_lsis,                               "LSIS Co., Ltd.");
    VENDOR_TEXT(ecvendor_statecore,                          "StateCore B.V.");
    VENDOR_TEXT(ecvendor_kj_infinity,                        "KJ-Infinity Enterprises Inc.");
    VENDOR_TEXT(ecvendor_chic,                               "Center of Human - centered Interaction for Coexistence(CHIC)");
    VENDOR_TEXT(ecvendor_littelfuse,                         "Littelfuse Selco A/S");
    VENDOR_TEXT(ecvendor_itr,                                "ITR GmbH Informationstechnologie Rauch");
    VENDOR_TEXT(ecvendor_massachusetts_amherst_university,   "University of Massachusetts at Amherst, Computer Science Department, Laboratory ");
    VENDOR_TEXT(ecvendor_pfeiffer,                           "Pfeiffer Vacuum SAS");
    VENDOR_TEXT(ecvendor_axor,                               "AXOR INDUSTRIES s.n.c.");
    VENDOR_TEXT(ecvendor_quadrep,                            "QuadRep Electronics (Taiwan) Ltd.");
    VENDOR_TEXT(ecvendor_herrmann,                           "Herrmann Ultraschalltechnik GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_precizika,                          "Precizika Metrology, UAB");
    VENDOR_TEXT(ecvendor_shanghai_recat,                     "Shanghai ReCAT Automation Control Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_u_and_r,                            "UandR GmbH Hardware- und Systemdesign");
    VENDOR_TEXT(ecvendor_xiamen,                             "XiaMen MicroControl Technology Co., Ltd");
    VENDOR_TEXT(ecvendor_oilgear,                            "The Oilgear Company");
    VENDOR_TEXT(ecvendor_mie,                                "MIE ELECTRONICS CO.,LTD. iSPC");
    VENDOR_TEXT(ecvendor_intech,                             "in-tech GmbH");
    VENDOR_TEXT(ecvendor_starflight,                         "Starflight Electronics");
    VENDOR_TEXT(ecvendor_ziehl_abegg,                        "ZIEHL-ABEGG SE");
    VENDOR_TEXT(ecvendor_ackermann,                          "Ackermann Automation GmbH");
    VENDOR_TEXT(ecvendor_helios,                             "Helios Technologies, Inc.");
    VENDOR_TEXT(ecvendor_italsensor,                         "Italsensor s.r.l.");
    VENDOR_TEXT(ecvendor_sartorius,                          "Sartorius Mechatronics CandD GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_evergrid,                           "Evergrid Solutions and Systems");
    VENDOR_TEXT(ecvendor_germanjet,                          "Germanjet Company Limited");
    VENDOR_TEXT(ecvendor_mapacode,                           "Mapacode Inc.");
    VENDOR_TEXT(ecvendor_biba,                               "BIBA - Bremer Institut fuer Produktion und Logistik GmbH");
    VENDOR_TEXT(ecvendor_texas_university,                   "The University of Texas at Austin");
    VENDOR_TEXT(ecvendor_nagano_oki,                         "Nagano Oki Electric Co., Ltd.");
    VENDOR_TEXT(ecvendor_condalo,                            "Condalo GmbH");
    VENDOR_TEXT(ecvendor_brooks,                             "Brooks Instrument, LLC");
    VENDOR_TEXT(ecvendor_florida_institute,                  "FLORIDA INSTITUTE FOR HUMAN and MACHINE COGNITION");
    VENDOR_TEXT(ecvendor_leica,                              "Leica Geosystems AG");
    VENDOR_TEXT(ecvendor_nabtesco,                           "Nabtesco Corporation");
    VENDOR_TEXT(ecvendor_bp_and_m,                           "BPandM Representations e Consultoria LTDA");
    VENDOR_TEXT(ecvendor_optronic,                           "Diamond Technologies, Inc.");
    VENDOR_TEXT(ecvendor_estun,                              "ESTUN AUTOMATION TECHNOLOGY CO.,LTD.");
    VENDOR_TEXT(ecvendor_ims,                                "IMS Messsysteme GmbH");
    VENDOR_TEXT(ecvendor_m_system,                           "M-System Co., Ltd.");
    VENDOR_TEXT(ecvendor_ferrotec,                           "Ferrotec (USA) Corporation - Temescal Division");
    VENDOR_TEXT(ecvendor_sick,                               "SICK IVP AB");
    VENDOR_TEXT(ecvendor_sinfoni,                            "SINFONIA TECHNOLOGY CO., LTD.");
    VENDOR_TEXT(ecvendor_pfeiffer_vacuum,                    "Pfeiffer Vacuum GmbH");
    VENDOR_TEXT(ecvendor_froude_hofmann,                     "Froude Hofmann Limited");
    VENDOR_TEXT(ecvendor_sabo,                               "SABO Elektronik GmbH");
    VENDOR_TEXT(ecvendor_bystronic,                          "Bystronic Laser AG");
    VENDOR_TEXT(ecvendor_invt,                               "INVT Industrial Technology (Shanghai) Co., Ltd.");
    VENDOR_TEXT(ecvendor_lumasense,                          "LumaSense Technologies GmbH");
    VENDOR_TEXT(ecvendor_bbh,                                "BBH Products GmbH");
    VENDOR_TEXT(ecvendor_hecht,                              "Hecht Automatisierungs-Systeme GmbH");
    VENDOR_TEXT(ecvendor_xelmo,                              "Xelmo AB");
    VENDOR_TEXT(ecvendor_carl_zeiss,                         "Carl Zeiss Industrielle Messtechnik GmbH");
    VENDOR_TEXT(ecvendor_genova_university,                  "University of Genova, Faculty of Engineering");
    VENDOR_TEXT(ecvendor_jot,                                "JOT Automation Oy");
    VENDOR_TEXT(ecvendor_seisakusho,                         "Sankyo Seisakusho Co.");
    VENDOR_TEXT(ecvendor_atv,                                "ATV-Elektronik Ges.m.b.H.");
    VENDOR_TEXT(ecvendor_panasonic,                          "Panasonic Industrial Devices SUNX Co., Ltd.");
    VENDOR_TEXT(ecvendor_ifm,                                "ifm electronic gmbh");
    VENDOR_TEXT(ecvendor_fisher,                             "Fisher Technical Services Inc.");
    VENDOR_TEXT(ecvendor_scle,                               "SCLE SFE");
    VENDOR_TEXT(ecvendor_higen,                              "HIGEN Motor Co., Ltd.");
    VENDOR_TEXT(ecvendor_baumer,                             "Baumer hhs GmbH");
    VENDOR_TEXT(ecvendor_moog_in,                            "Moog Inc.");
    VENDOR_TEXT(ecvendor_xios,                               "XIOS Hogeschool Limburg, Department N-Technology");
    VENDOR_TEXT(ecvendor_azbil,                              "Azbil Corporation");
    VENDOR_TEXT(ecvendor_delta_tau,                          "Delta Tau Data Systems, Inc.");
    VENDOR_TEXT(ecvendor_heraeus,                            "Heraeus Electro-Nite International N.V.");
    VENDOR_TEXT(ecvendor_esw,                                "ESW GmbH");
    VENDOR_TEXT(ecvendor_cg_drives,                          "CG Drives and Automation AB");
    VENDOR_TEXT(ecvendor_procom,                             "ProCom GmbH");
    VENDOR_TEXT(ecvendor_alstom,                             "ALSTOM Grid SAS - Systems");
    VENDOR_TEXT(ecvendor_robot,                              "Robot Makers GmbH");
    VENDOR_TEXT(ecvendor_brooks_automation,                  "Brooks Automation, Inc");
    VENDOR_TEXT(ecvendor_hitachi_metals,                     "Hitachi Metals Ltd., Piping Components Company");
    VENDOR_TEXT(ecvendor_interroll,                          "Interroll Automation GmbH");
    VENDOR_TEXT(ecvendor_ckd,                                "CKD Corporation");
    VENDOR_TEXT(ecvendor_stiwa,                              "STIWA Automation GmbH");
    VENDOR_TEXT(ecvendor_tpa,                                "T.P.A. S.p.A");
    VENDOR_TEXT(ecvendor_guodian_nanjing,                    "Guodian Nanjing Automation Co., Ltd.");
    VENDOR_TEXT(ecvendor_prosoft,                            "Prosoft-Systems Ltd");
    VENDOR_TEXT(ecvendor_polytype,                           "Polytype SA");
    VENDOR_TEXT(ecvendor_sensodrive,                         "SENSODRIVE GmbH");
    VENDOR_TEXT(ecvendor_delta,                              "Delta Computer Systems, Inc.");
    VENDOR_TEXT(ecvendor_friedrich_luetze,                   "Friedrich Luetze GmbH");
    VENDOR_TEXT(ecvendor_compressor_controls,                "Compressor Controls Corporation");
    VENDOR_TEXT(ecvendor_diamond_light,                      "Diamond Light Source Limited");
    VENDOR_TEXT(ecvendor_beckman,                            "Beckman Coulter, Inc.");
    VENDOR_TEXT(ecvendor_allied_motion,                      "Allied Motion Technologies, Inc.");
    VENDOR_TEXT(ecvendor_nor_cal,                            "Nor-Cal Products, Inc.");
    VENDOR_TEXT(ecvendor_automata,                           "AUTOMATA GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_fraunhofer_institut_IWS,            "Fraunhofer Institut fuer Werkstoff- und Strahlentechnik IWS");
    VENDOR_TEXT(ecvendor_inficon,                            "INFICON AG");
    VENDOR_TEXT(ecvendor_hexagon,                            "Hexagon Metrology GmbH");
    VENDOR_TEXT(ecvendor_shimadzu,                           "Shimadzu Corporation");
    VENDOR_TEXT(ecvendor_dasa,                               "Dasa Control Systems AB");
    VENDOR_TEXT(ecvendor_shoei,                              "SHOEI Electric Co., Ltd.");
    VENDOR_TEXT(ecvendor_progressio,                         "Progressio, LLC");
    VENDOR_TEXT(ecvendor_maccon,                             "MACCON GmbH");
    VENDOR_TEXT(ecvendor_moog_ireland,                       "Moog Ireland, Ltd.");
    VENDOR_TEXT(ecvendor_espera,                             "ESPERA-WERKE GMBH");
    VENDOR_TEXT(ecvendor_w_plus_r,                           "Automation W+R GmbH");
    VENDOR_TEXT(ecvendor_oceaneering,                        "Oceaneering Space Systems");
    VENDOR_TEXT(ecvendor_eostech,                            "EOStech S.r.l.");
    VENDOR_TEXT(ecvendor_baptiste_de_baudre,                 "Lycee Jean-Baptiste de Baudre");
    VENDOR_TEXT(ecvendor_banja_luka_university,              "University of Banja Luka");
    VENDOR_TEXT(ecvendor_eding,                              "Eding CNC");
    VENDOR_TEXT(ecvendor_zuehlke,                            "Zuehlke Engineering AG");
    VENDOR_TEXT(ecvendor_addiva,                             "Addiva Consulting AB");
    VENDOR_TEXT(ecvendor_pteris,                             "Pteris Global Limited");
    VENDOR_TEXT(ecvendor_chaos,                              "Chaos Technology");
    VENDOR_TEXT(ecvendor_tokyo_institute,                    "Tokyo Institute of Technology, Hirose Fukushima Lab.");
    VENDOR_TEXT(ecvendor_seichter,                           "Seichter GmbH");
    VENDOR_TEXT(ecvendor_motion_control,                     "Motion Control Systems, Inc.");
    VENDOR_TEXT(ecvendor_moog_nl,                            "Moog B.V. in the Netherlands");
    VENDOR_TEXT(ecvendor_kinlo,                              "Kinlo Technology and System (Shenzhen) Co.,Ltd.");
    VENDOR_TEXT(ecvendor_cellsystems,                        "CellSystems, LLC");
    VENDOR_TEXT(ecvendor_shinano_kenshi,                     "Shinano Kenshi Co., Ltd.");
    VENDOR_TEXT(ecvendor_micro_epsilon,                      "MICRO-EPSILON MESSTECHNIK GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_viable,                             "Viable Bytes, Inc.");
    VENDOR_TEXT(ecvendor_kontron,                            "Kontron AG");
    VENDOR_TEXT(ecvendor_ik4,                                "IK4-IKERLAN");
    VENDOR_TEXT(ecvendor_hmk,                                "hmk Daten-System-Technik GmbH");
    VENDOR_TEXT(ecvendor_iit_institute,                      "Istituto Italiano di Tecnologia (IIT)");
    VENDOR_TEXT(ecvendor_qingdao_incman,                     "Qingdao INCMAN Robot Co., Ltd.");
    VENDOR_TEXT(ecvendor_kashiyama,                          "Kashiyama Industries, Ltd.");
    VENDOR_TEXT(ecvendor_tg_drives,                          "TG Drives s.r.o.");
    VENDOR_TEXT(ecvendor_watlow,                             "Watlow Electric Manufacturing Company");
    VENDOR_TEXT(ecvendor_synertronixx,                       "synertronixx GmbH");
    VENDOR_TEXT(ecvendor_batalpha,                           "batalpha Bobach GmbH");
    VENDOR_TEXT(ecvendor_edwards,                            "Edwards Limited");
    VENDOR_TEXT(ecvendor_engel_au,                           "ENGEL AUSTRIA GmbH");
    VENDOR_TEXT(ecvendor_fujikin,                            "Fujikin Incorporated");
    VENDOR_TEXT(ecvendor_comet,                              "COMET Technologies USA, Inc.");
    VENDOR_TEXT(ecvendor_schleuniger,                        "Schleuniger AG");
    VENDOR_TEXT(ecvendor_panasonic_automotive_systems,       "Panasonic Corporation, Automotive and Industrial Systems Company");
    VENDOR_TEXT(ecvendor_tangshan_kaiyuan,                   "TangShan Kaiyuan Welding Automation Technology Institute Co., Ltd.");
    VENDOR_TEXT(ecvendor_solectrix,                          "Solectrix GmbH");
    VENDOR_TEXT(ecvendor_cloud_state_university,             "St. Cloud State University, Electrical and Computer Engineering Department");
    VENDOR_TEXT(ecvendor_jlg,                                "JLG AUTOMATION BVBA");
    VENDOR_TEXT(ecvendor_burckhardt,                         "Burckhardt Compression AG");
    VENDOR_TEXT(ecvendor_rong_shun_xuan,                     "Rong Shun Xuan Corp.");
    VENDOR_TEXT(ecvendor_balluff_stm,                        "Balluff STM GmbH");
    VENDOR_TEXT(ecvendor_endress_hauser,                     "Endress+Hauser Flowtec AG");
    VENDOR_TEXT(ecvendor_motor_power_company,                "Motor Power Company S.r.l.");
    VENDOR_TEXT(ecvendor_ebi,                                "EBI Electric Inc.");
    VENDOR_TEXT(ecvendor_hs_luzern,                          "Hochschule Luzern - Technik and Architektur");
    VENDOR_TEXT(ecvendor_ge_global,                          "GE Global Research");
    VENDOR_TEXT(ecvendor_universiteit_leuven,                "Katholieke Universiteit Leuven, Department of Mechanical Engineering");
    VENDOR_TEXT(ecvendor_ectronic,                           "Ectronic GmbH");
    VENDOR_TEXT(ecvendor_bundesamt,                          "Bundesamt fuer Wehrtechnik und Beschaffung, Dienststelle WTD 81");
    VENDOR_TEXT(ecvendor_daihen,                             "DAIHEN Corporation");
    VENDOR_TEXT(ecvendor_hcl,                                "HCL Technologies Ltd.");
    VENDOR_TEXT(ecvendor_three_t,                            "3T B.V.");
    VENDOR_TEXT(ecvendor_eindhoven_university,               "Eindhoven University of Technology");
    VENDOR_TEXT(ecvendor_innovent,                           "INNOVENT e.V.");
    VENDOR_TEXT(ecvendor_surrey,                             "Surrey Satellite Technology Limited");
    VENDOR_TEXT(ecvendor_ametek,                             "AMETEK Programmable Power, Inc.");
    VENDOR_TEXT(ecvendor_engleder,                           "engleder embedded");
    VENDOR_TEXT(ecvendor_pusan,                              "Pusan National University");
    VENDOR_TEXT(ecvendor_eth_zuerich,                        "ETH Zuerich, Institute of Robotics and Intelligent Systems");
    VENDOR_TEXT(ecvendor_berner_mattner,                     "Berner and Mattner Systemtechnik GmbH");
    VENDOR_TEXT(ecvendor_horiba_ltd,                         "HORIBA, Ltd.");
    VENDOR_TEXT(ecvendor_changwon_university,                "Changwon National University, College of Engineering, Department of Electrical Engineering");
    VENDOR_TEXT(ecvendor_penko,                              "Penko Engineering B.V.");
    VENDOR_TEXT(ecvendor_fujitsu,                            "Fujitsu Semiconductor Europe GmbH");
    VENDOR_TEXT(ecvendor_mirle,                              "Mirle Automation Corporation");
    VENDOR_TEXT(ecvendor_fanuc,                              "FANUC CORPORATION");
    VENDOR_TEXT(ecvendor_kse,                                "KSE GmbH");
    VENDOR_TEXT(ecvendor_euchner,                            "EUCHNER GmbH + Co. KG");
    VENDOR_TEXT(ecvendor_benjamin,                           "benjamin GmbH");
    VENDOR_TEXT(ecvendor_hans_laser,                         "Han's Laser Technology Co.,Ltd.");
    VENDOR_TEXT(ecvendor_guangdong_uni_automation,           "Guangdong University of Technology, Faculty of Automation");
    VENDOR_TEXT(ecvendor_de_santa_catarina,                  "Instituto Federal de Santa Catarina");
    VENDOR_TEXT(ecvendor_instrutech,                         "InstruTech Inc.");
    VENDOR_TEXT(ecvendor_ktl,                                "KTL Corporation");
    VENDOR_TEXT(ecvendor_hs_pforzheim,                       "Hochschule Pforzheim, Fakultaet fuer Technik");
    VENDOR_TEXT(ecvendor_ewm,                                "EWM HIGHTEC WELDING GmbH");
    VENDOR_TEXT(ecvendor_jilin_yongda,                       "Jilin Yongda Group Company Ltd.");
    VENDOR_TEXT(ecvendor_arrow,                              "Arrow Central Europe GmbH");
    VENDOR_TEXT(ecvendor_phoseon,                            "Phoseon Technology");
    VENDOR_TEXT(ecvendor_item,                               "item Industrietechnik GmbH");
    VENDOR_TEXT(ecvendor_shanghai_inno_drive,                "Shanghai Inno-drive Electric Co., Ltd.");
    VENDOR_TEXT(ecvendor_wuhan,                              "Wuhan University of Technology, School of Automation");
    VENDOR_TEXT(ecvendor_advanet,                            "Advanet Inc.");
    VENDOR_TEXT(ecvendor_wandercraft,                        "Wandercraft SAS");
    VENDOR_TEXT(ecvendor_changzhou_xiangyun,                 "Changzhou Xiangyun Monitoring Software Co., Ltd.");
    VENDOR_TEXT(ecvendor_santest,                            "SANTEST CO., LTD.");
    VENDOR_TEXT(ecvendor_entesys,                            "EnTeSys GmbH");
    VENDOR_TEXT(ecvendor_lot,                                "LOT Vacuum Co., Ltd.");
    VENDOR_TEXT(ecvendor_asm,                                "ASM America Inc.");
    VENDOR_TEXT(ecvendor_taiwan_pulse_motion,                "Taiwan Pulse Motion Co. Ltd.");
    VENDOR_TEXT(ecvendor_cni,                                "CNi Informatica S.r.l.");
    VENDOR_TEXT(ecvendor_enfas,                              "enfas GmbH");
    VENDOR_TEXT(ecvendor_megmeet_drive,                      "Shenzhen Megmeet Drive Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_danish_aerospace,                   "Danish Aerospace Company");
    VENDOR_TEXT(ecvendor_panasonic_production,               "Panasonic Production Engineering Co., Ltd.");
    VENDOR_TEXT(ecvendor_aradex,                             "ARADEX AG");
    VENDOR_TEXT(ecvendor_toyogiken,                          "TOYOGIKEN CO.,LTD.");
    VENDOR_TEXT(ecvendor_zao_trascon,                        "ZAO Trascon Technology");
    VENDOR_TEXT(ecvendor_arem,                               "AREM PRO, s.r.o.");
    VENDOR_TEXT(ecvendor_googol,                             "Googol Technology (HK) Ltd.");
    VENDOR_TEXT(ecvendor_vecna,                              "Vecna Technologies, Inc.");
    VENDOR_TEXT(ecvendor_tu_dresden,                         "Technische Universitaet Dresden, Fakultaet Elektrotechnik und Informationstechnik");
    VENDOR_TEXT(ecvendor_axxon,                              "Axxon Computer Corporation");
    VENDOR_TEXT(ecvendor_beijing_motrotech,                  "Beijing Motrotech Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_woehner,                            "Worhner GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_hangzhou_tongling,                  "Hangzhou Tongling Automation Co., Ltd.");
    VENDOR_TEXT(ecvendor_audix,                              "Audix Corporation");
    VENDOR_TEXT(ecvendor_tu_wien,                            "Technische Universitaet Wien, Fakultaet fuer Elektrotechnik und Informationstechnik");
    VENDOR_TEXT(ecvendor_areva_np,                           "AREVA NP");
    VENDOR_TEXT(ecvendor_taurus_instruments,                 "TAURUS instruments GmbH");
    VENDOR_TEXT(ecvendor_aveox,                              "Aveox Inc.");
    VENDOR_TEXT(ecvendor_asml,                               "ASML Holding N.V.");
    VENDOR_TEXT(ecvendor_haslerrail,                         "HaslerRail AG");
    VENDOR_TEXT(ecvendor_intek,                              "Intek Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_crouzet,                            "Crouzet Automatismes");
    VENDOR_TEXT(ecvendor_joshua,                             "Joshua 1 Systems Inc.");
    VENDOR_TEXT(ecvendor_artech,                             "Artech Electronics Co., Ltd.");
    VENDOR_TEXT(ecvendor_fuji_machinery,                     "FUJI MACHINERY CO.,LTD.");
    VENDOR_TEXT(ecvendor_foshan_logen,                       "FoShan Logen Robotics Co., Ltd.");
    VENDOR_TEXT(ecvendor_dvdb,                               "DVDB-electronics bvba");
    VENDOR_TEXT(ecvendor_tess,                               "Tool Express-Service Schraubertechnik GmbH (TESS GmbH)");
    VENDOR_TEXT(ecvendor_king_giants,                        "King Giants Precision Industry Co., Ltd.");
    VENDOR_TEXT(ecvendor_panax,                              "PANAX SYSTEM Co., Ltd.");
    VENDOR_TEXT(ecvendor_hitachi_eu,                         "Hitachi Europe GmbH");
    VENDOR_TEXT(ecvendor_zdauto_azdauto,                     "ZDAUTO AZDAUTO Automation Technology Co., Ltd. utomation Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_temis,                              "Temis S.r.l.");
    VENDOR_TEXT(ecvendor_daikin,                             "DAIKIN INDUSTRIES, LTD., Oil Hydraulics Division");
    VENDOR_TEXT(ecvendor_avalue,                             "Avalue Technology Inc.");
    VENDOR_TEXT(ecvendor_ldz,                                "LDZ Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_eletech,                            "Eletech S.r.l.");
    VENDOR_TEXT(ecvendor_portwell,                           "Portwell, Inc.");
    VENDOR_TEXT(ecvendor_shenzhen_sine,                      "Shenzhen Sine Electric Co., Ltd");
    VENDOR_TEXT(ecvendor_hildebrandt,                        "HIMA Paul Hildebrandt GmbH");
    VENDOR_TEXT(ecvendor_covidien,                           "Covidien LP");
    VENDOR_TEXT(ecvendor_weintek,                            "Weintek Labs., Inc.");
    VENDOR_TEXT(ecvendor_mitwell,                            "MITWELL Inc.");
    VENDOR_TEXT(ecvendor_dorna,                              "DORNA Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_am_consulting,                      "A M Consulting");
    VENDOR_TEXT(ecvendor_genesi,                             "GENESI ELETTRONICA Srl");
    VENDOR_TEXT(ecvendor_molex,                              "Molex Canada Limited");
    VENDOR_TEXT(ecvendor_vanguard,                           "Vanguard Systems Inc.");
    VENDOR_TEXT(ecvendor_ifatos,                             "ifatos GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_ingenieria,                         "Ingenieria UNO S.L");
    VENDOR_TEXT(ecvendor_daekhon,                            "Daekhon Corporation");
    VENDOR_TEXT(ecvendor_marubeni,                           "Marubeni Information Systems Co., Ltd.");
    VENDOR_TEXT(ecvendor_ydk,                                "YDK Co., Ltd.");
    VENDOR_TEXT(ecvendor_eta,                                "E-T-A Elektrotechnische Apparate GmbH");
    VENDOR_TEXT(ecvendor_justech,                            "Justech Precision Industry Co., Ltd.");
    VENDOR_TEXT(ecvendor_vollmer_werke,                      "VOLLMER WERKE Maschinenfabrik GmbH");
    VENDOR_TEXT(ecvendor_hs_georg_simon_ohm,                 "Technische Hochschule Nuernberg Georg Simon Ohm");
    VENDOR_TEXT(ecvendor_gene,                               "Gene Automation Technology Ltd.");
    VENDOR_TEXT(ecvendor_weihai_zheng,                       "Weihai Zheng Qi Mechatronics Technology Ltd.");
    VENDOR_TEXT(ecvendor_intervalzero,                       "IntervalZero, Inc.");
    VENDOR_TEXT(ecvendor_axonim,                             "AXONIM LLC");
    VENDOR_TEXT(ecvendor_seoul_university,                   "University of Seoul, College of Engineering, Department of Mechanical and Information Engineering");
    VENDOR_TEXT(ecvendor_professional,                       "Professional Computer Technology Limited");
    VENDOR_TEXT(ecvendor_shenyang_golding,                   "Shenyang Golding NC and Intelligence Tech Co., Ltd.");
    VENDOR_TEXT(ecvendor_iwaki,                              "IWAKI CO., LTD.");
    VENDOR_TEXT(ecvendor_trp,                                "TRP Engineering College, Department of Electronics and Communication Engineering (ECE)");
    VENDOR_TEXT(ecvendor_epi_elettronica,                    "EPI elettronica s.a.s.");
    VENDOR_TEXT(ecvendor_iram,                               "Institut de RadioAstronomie Millimetrique");
    VENDOR_TEXT(ecvendor_tu_graz,                            "Technische Universitaet Graz, Fakultaet fuer Maschinenbau und Wirtschaftswissenschaften");
    VENDOR_TEXT(ecvendor_probeam,                            "pro-beam AG and Co. KGaA");
    VENDOR_TEXT(ecvendor_servotechnica,                      "Servotechnica ZAO");
    VENDOR_TEXT(ecvendor_hanwha,                             "Hanwha Techwin");
    VENDOR_TEXT(ecvendor_g_and_s,                            "GandS Intelligent Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_schaeffler,                         "Schaeffler Engineering");
    VENDOR_TEXT(ecvendor_basque_university,                  "University of the Basque Country, Faculty of Engineering, Department of Electronics and Telecommunications");
    VENDOR_TEXT(ecvendor_arges,                              "ARGES GmbH");
    VENDOR_TEXT(ecvendor_control_chief,                      "Control Chief Corporation");
    VENDOR_TEXT(ecvendor_konplan,                            "konplan systemhaus ag");
    VENDOR_TEXT(ecvendor_embex,                              "embeX GmbH");
    VENDOR_TEXT(ecvendor_cosmotechs,                         "COSMOTECHS Co., Ltd");
    VENDOR_TEXT(ecvendor_dynamic,                            "Dynamic Systems Inc.");
    VENDOR_TEXT(ecvendor_semikron,                           "SEMIKRON Elektronik GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_wuxi_ivoyage,                       "Wuxi Ivoyage Control Technology CO.,LTD");
    VENDOR_TEXT(ecvendor_caf,                                "CAF Signalling S.L.");
    VENDOR_TEXT(ecvendor_smart,                              "SMART Electronic Development GmbH");
    VENDOR_TEXT(ecvendor_yokogawa,                           "Yokogawa Electric Corporation");
    VENDOR_TEXT(ecvendor_norwegian_university,               "Norwegian University of Science and Technoloogy");
    VENDOR_TEXT(ecvendor_robostar,                           "Robostar Co., Ltd");
    VENDOR_TEXT(ecvendor_trumpf_werkzeugmaschinen,           "TRUMPF Werkzeugmaschinen GmbH + Co. KG");
    VENDOR_TEXT(ecvendor_high_performance,                   "High Performance Motion System Development Co., Ltd.");
    VENDOR_TEXT(ecvendor_kozaka,                             "Kozaka Electronic Design Inc.");
    VENDOR_TEXT(ecvendor_aeronautical_systems,               "Aeronautical Systems Engineering Inc.");
    VENDOR_TEXT(ecvendor_agile,                              "Agile Planet, Inc.");
    VENDOR_TEXT(ecvendor_hon_hai,                            "Hon Hai Precision Industry Co., Ltd.");
    VENDOR_TEXT(ecvendor_systeme_plus_steuerungen,           "Systeme + Steuerungen GmbH");
    VENDOR_TEXT(ecvendor_huron,                              "Huron Net Works, Inc.");
    VENDOR_TEXT(ecvendor_souther_switzerland_dti,            "University of Applied Sciences of Southern Switzerland, Department of Innovative Technologies (DTI)");
    VENDOR_TEXT(ecvendor_mako,                               "MAKO Surgical Corp.");
    VENDOR_TEXT(ecvendor_rainer_thomas,                      "Rainer Thomas Messtechnik GmbH");
    VENDOR_TEXT(ecvendor_eltek,                              "ELTEK spol. s r.o.");
    VENDOR_TEXT(ecvendor_vecow,                              "Vecow Co., Ltd.");
    VENDOR_TEXT(ecvendor_m_and_p,                            "MandP Motion Control and Power Electronics GmbH");
    VENDOR_TEXT(ecvendor_leybold,                            "Leybold GmbH");
    VENDOR_TEXT(ecvendor_panasonic_america,                  "Panasonic Industrial Devices Sales Company of America");
    VENDOR_TEXT(ecvendor_eilersen,                           "Eilersen Electric Digital Systems A/S");
    VENDOR_TEXT(ecvendor_inter,                              "Inter Factory Partners Co., LTD.");
    VENDOR_TEXT(ecvendor_power_instrument,                   "Power Instrument Co., Ltd.");
    VENDOR_TEXT(ecvendor_coswort,                            "Cosworth Group Holdings Ltd");
    VENDOR_TEXT(ecvendor_south_china_uni_mae,                "South China University of Technology, School of Mechanical and Automotive Engineering");
    VENDOR_TEXT(ecvendor_warwick,                            "WARWICK INSTRUMENTS LTD.");
    VENDOR_TEXT(ecvendor_shenzhen_invt,                      "Shenzhen INVT Co., Ltd.");
    VENDOR_TEXT(ecvendor_wiedeg,                             "WIEDEG Elektronik GmbH");
    VENDOR_TEXT(ecvendor_nke,                                "NKE corporation");
    VENDOR_TEXT(ecvendor_madrid_university,                  "Universidad Politecnica de Madrid");
    VENDOR_TEXT(ecvendor_bot_and_dolly,                      "Bot and Dolly");
    VENDOR_TEXT(ecvendor_recif,                              "RECIF Technologies");
    VENDOR_TEXT(ecvendor_ati,                                "ATI Industrial Automation");
    VENDOR_TEXT(ecvendor_advantest,                          "ADVANTEST CORPORATION");
    VENDOR_TEXT(ecvendor_multivac,                           "MULTIVAC Sepp Haggenmueller SE and Co. KG");
    VENDOR_TEXT(ecvendor_roland,                             "ROLAND ELECTRONIC GmbH");
    VENDOR_TEXT(ecvendor_contec,                             "CONTEC Co., Ltd.");
    VENDOR_TEXT(ecvendor_alizem,                             "Alizem Inc.");
    VENDOR_TEXT(ecvendor_chyng_hong,                         "Chyng Hong Electronic Co., Ltd.");
    VENDOR_TEXT(ecvendor_yaskawa_america,                    "Yaskawa America Inc.");
    VENDOR_TEXT(ecvendor_georg_schlegel,                     "Georg Schlegel GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_faraday,                            "Faraday Technology Corporation");
    VENDOR_TEXT(ecvendor_sipro,                              "SIPRO S.r.l.");
    VENDOR_TEXT(ecvendor_twente_university,                  "University of Twente, Faculty of Engineering Technology (CTW)");
    VENDOR_TEXT(ecvendor_aalborg_university,                 "Aalborg University");
    VENDOR_TEXT(ecvendor_s_sys_bvba,                         "S-SYS bvba");
    VENDOR_TEXT(ecvendor_ningbo_mingpu,                      "Ningbo Mingpu Automation Technology Co.");
    VENDOR_TEXT(ecvendor_elrest,                             "elrest Automationssysteme GmbH");
    VENDOR_TEXT(ecvendor_kyosan,                             "Kyosan Electric Manufacturing Co., Ltd.");
    VENDOR_TEXT(ecvendor_custom,                             "Custom Machines");
    VENDOR_TEXT(ecvendor_kfm,                                "KFM Regelungstechnik GmbH");
    VENDOR_TEXT(ecvendor_ishida,                             "ISHIDA CO., LTD.");
    VENDOR_TEXT(ecvendor_beijing_university,                 "Beijing University of Technology");
    VENDOR_TEXT(ecvendor_hannifin,                           "Hannifin Corporation, Electromechanical Automation Division, North America");
    VENDOR_TEXT(ecvendor_tsino_dynatron,                     "Tsino-dynatron Electrical Technology Beijing Co., Ltd.");
    VENDOR_TEXT(ecvendor_penta,                              "PENTA TRADING Spol. S.r.o.");
    VENDOR_TEXT(ecvendor_china_university,                   "University of Electronic Science and Technology of China, School of Optoelectronic Information");
    VENDOR_TEXT(ecvendor_altus,                              "Altus Sistemas de Informatica S/A");
    VENDOR_TEXT(ecvendor_sanming,                            "Sanming University, Sanming Mechanical CAD Engineering Research Center");
    VENDOR_TEXT(ecvendor_jenoptik,                           "JENOPTIK Industrial Metrology Germany");
    VENDOR_TEXT(ecvendor_beijing_knd,                        "Beijing KND CNC Technique Co., Ltd.");
    VENDOR_TEXT(ecvendor_jw_shannon,                         "JW Shannon Engineers");
    VENDOR_TEXT(ecvendor_a2v,                                "A2V Mecatronique SAS");
    VENDOR_TEXT(ecvendor_nexcom,                             "NEXCOM International Co., Ltd.");
    VENDOR_TEXT(ecvendor_jiangyin_huafeng,                   "Jiangyin Huafeng Printing Machinery Co., Ltd.");
    VENDOR_TEXT(ecvendor_lectra,                             "Lectra");
    VENDOR_TEXT(ecvendor_beijer,                             "Beijer Electronics Products AB");
    VENDOR_TEXT(ecvendor_cj_hartman,                         "C J Hartman Elektronik AB");
    VENDOR_TEXT(ecvendor_hurco,                              "Hurco Automation Ltd.");
    VENDOR_TEXT(ecvendor_autonics,                           "Autonics Corporation");
    VENDOR_TEXT(ecvendor_brom,                               "Brom Mechatronica B.V.");
    VENDOR_TEXT(ecvendor_vrije_university,                   "Vrije Universiteit Brussel, Faculty of Engineering");
    VENDOR_TEXT(ecvendor_alluris,                            "Alluris GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_hs_offenburg,                       "Hochschule Offenburg, Fakultaet Elektrotechnik und Informationstechnik");
    VENDOR_TEXT(ecvendor_heidolph,                           "Heidolph Elektro GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_ge_transportation,                  "GE Transportation");
    VENDOR_TEXT(ecvendor_maex,                               "MAE,X GmbH");
    VENDOR_TEXT(ecvendor_durst,                              "Durst Phototechnik Digital Technology GmbH");
    VENDOR_TEXT(ecvendor_omsk_university,                    "Omsk State Technical University, Department of AElectricity industry");
    VENDOR_TEXT(ecvendor_embedded_bonjour,                   "Embedded-Bonjour GmbH");
    VENDOR_TEXT(ecvendor_mettler_toledo,                     "Mettler-Toledo Garvens GmbH");
    VENDOR_TEXT(ecvendor_renesas,                            "Renesas Electronics Corp.");
    VENDOR_TEXT(ecvendor_lju,                                "LJU Automatisierungstechnik GmbH");
    VENDOR_TEXT(ecvendor_eura_drives,                        "EURA DRIVES ELECTRIC CO. LTD");
    VENDOR_TEXT(ecvendor_eutron,                             "EUTRON S.p.A.");
    VENDOR_TEXT(ecvendor_flanders,                           "FLANDERS Inc.");
    VENDOR_TEXT(ecvendor_digital_dynamics,                   "Digital Dynamics, Inc.");
    VENDOR_TEXT(ecvendor_ge_medical,                         "GE Medical Systems Europe");
    VENDOR_TEXT(ecvendor_physik_instumente,                  "Physik Instumente GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_hypertherm,                         "Hypertherm Inc.");
    VENDOR_TEXT(ecvendor_hrid,                               "HRID d.o.o.");
    VENDOR_TEXT(ecvendor_aotai,                              "Aotai Electric Co., LTD");
    VENDOR_TEXT(ecvendor_control_concepts,                   "Control Concepts Inc.");
    VENDOR_TEXT(ecvendor_jovetech,                           "JoveTech Co., Ltd.");
    VENDOR_TEXT(ecvendor_pohang_university,                  "POHANG UNIVERSITY OF SCIENCE AND TECHNOLOGY, Department of Electrical Engineering");
    VENDOR_TEXT(ecvendor_cyclomedia,                         "CycloMedia Technology B.V.");
    VENDOR_TEXT(ecvendor_sun_tectro,                         "SUN-TECTRO LTD.");
    VENDOR_TEXT(ecvendor_jiao_tong_university,               "Shanghai Jiao Tong University, School of Electronic Information and Electrical Engineering");
    VENDOR_TEXT(ecvendor_koenig,                             "koenig-pa GmbH");
    VENDOR_TEXT(ecvendor_anton_paar,                         "Anton Paar TriTec SA");
    VENDOR_TEXT(ecvendor_v_tex,                              "V TEX Corporation");
    VENDOR_TEXT(ecvendor_edge_technologies,                  "Edge Technologies");
    VENDOR_TEXT(ecvendor_ebara,                              "EBARA CORPORATION");
    VENDOR_TEXT(ecvendor_buffalo_university,                 "University at Buffalo");
    VENDOR_TEXT(ecvendor_aurotek,                            "Aurotek Corporation");
    VENDOR_TEXT(ecvendor_blubit,                             "Blubit d.o.o.");
    VENDOR_TEXT(ecvendor_toplens,                            "Toplens Hangzhou, Inc.");
    VENDOR_TEXT(ecvendor_chung_cheng_university,             "National Chung Cheng University");
    VENDOR_TEXT(ecvendor_hexagon_tech_centre,                "Hexagon Technology Center GmbH");
    VENDOR_TEXT(ecvendor_graph_tech,                         "Graph-Tech AG");
    VENDOR_TEXT(ecvendor_lanthan,                            "Lanthan GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_nucleus,                            "Nucleus GmbH");
    VENDOR_TEXT(ecvendor_stratec,                            "STRATEC CONTROL-SYSTEMS GmbH");
    VENDOR_TEXT(ecvendor_wien_university,                    "Universitaet Wien, Fakultaet fuer Physik, Isotopenforschung");
    VENDOR_TEXT(ecvendor_inno_spec,                          "inno-spec GmbH");
    VENDOR_TEXT(ecvendor_thyssenkrupp,                       "ThyssenKrupp Presta AG");
    VENDOR_TEXT(ecvendor_denso_wave,                         "DENSO WAVE INCORPORATED");
    VENDOR_TEXT(ecvendor_eurosoft,                           "EuroSoft S.r.l.");
    VENDOR_TEXT(ecvendor_british_columbia_university,        "University of British Columbia, Faculty of Applied Science, Department of Mechanical Engineering");
    VENDOR_TEXT(ecvendor_renergy_tianjin,                    "REnergy Electric Tianjin Ltd.");
    VENDOR_TEXT(ecvendor_newyoungsystem,                     "NewYoungSystem Co., Ltd.");
    VENDOR_TEXT(ecvendor_selema,                             "Selema S.r.l.");
    VENDOR_TEXT(ecvendor_reading_university,                 "University of Reading, School of Systems Engineering");
    VENDOR_TEXT(ecvendor_koganei,                            "KOGANEI CORPORATION");
    VENDOR_TEXT(ecvendor_mazet,                              "MAZeT GmbH");
    VENDOR_TEXT(ecvendor_nottingham_university,              "The University of Nottingham, Faculty of Engineering, Electrical Systems and Optics Research Division");
    VENDOR_TEXT(ecvendor_quanta,                             "Quanta Storage Inc.");
    VENDOR_TEXT(ecvendor_azbil_taishin,                      "Azbil Taishin Co., Ltd.");
    VENDOR_TEXT(ecvendor_relitech,                           "Relitech B.V.");
    VENDOR_TEXT(ecvendor_dhpc,                               "DHPC Technologies, Inc.");
    VENDOR_TEXT(ecvendor_jordan_valley,                      "Jordan Valley Semiconductors Ltd.");
    VENDOR_TEXT(ecvendor_microcreate,                        "MicroCreate System Co., Ltd.");
    VENDOR_TEXT(ecvendor_ab_and_t,                           "ABandT S.r.l.");
    VENDOR_TEXT(ecvendor_t3lab,                              "T3LAB - Technology Transfer Team");
    VENDOR_TEXT(ecvendor_coptonix,                           "Coptonix GmbH");
    VENDOR_TEXT(ecvendor_karl_mayer,                         "KARL MAYER Textilmaschinenfabrik GmbH");
    VENDOR_TEXT(ecvendor_inoson,                             "inoson GmbH");
    VENDOR_TEXT(ecvendor_ge_power,                           "GE Power and Water Distributed Power");
    VENDOR_TEXT(ecvendor_shanghai_yuanzhi,                   "Shanghai Yuanzhi Robot Co., Ltd.");
    VENDOR_TEXT(ecvendor_oyo,                                "OYO ELECTRIC CO., LTD.");
    VENDOR_TEXT(ecvendor_solwit,                             "Solwit SA");
    VENDOR_TEXT(ecvendor_jabil_circuit,                      "Jabil Circuit, Inc.");
    VENDOR_TEXT(ecvendor_renesas_semiconductor,              "Renesas Semiconductor Package and Test Solutions Co., Ltd.");
    VENDOR_TEXT(ecvendor_tu_berlin,                          "Technische Universitaet Berlin, Fakultaet Verkehrs- und Maschinensysteme");
    VENDOR_TEXT(ecvendor_mettler_toledo_changzhou,           "Mettler-Toledo (Changzhou) Precision Instrument Ltd.");
    VENDOR_TEXT(ecvendor_sentronic,                          "Sentronic International Corp.");
    VENDOR_TEXT(ecvendor_leas,                               "LEAS (Laboratoire d'electronique Angelidis et Sarrault)");
    VENDOR_TEXT(ecvendor_mapna,                              "MAPNA Electric and Control, Engineering and Manufacturing Co.");
    VENDOR_TEXT(ecvendor_nettechnix_e_and_p,                 "NetTechnix EandP GmbH");
    VENDOR_TEXT(ecvendor_integrated_dynamics,                "Integrated Dynamics Engineering GmbH");
    VENDOR_TEXT(ecvendor_toho,                               "Toho Technology Corporation");
    VENDOR_TEXT(ecvendor_salvagnini,                         "Salvagnini Italia S.p.A.");
    VENDOR_TEXT(ecvendor_shanghai_triowin,                   "Shanghai Triowin Automation Machinery Co., Ltd.");
    VENDOR_TEXT(ecvendor_hytec,                              "Hytec Electronics Ltd.");
    VENDOR_TEXT(ecvendor_xian_xiangxun,                      "XiAN Xiangxun Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_schmidiger,                         "Schmidiger GmbH");
    VENDOR_TEXT(ecvendor_master,                             "MASTER LTD");
    VENDOR_TEXT(ecvendor_korea_university,                   "Korea University, College of Engineering");
    VENDOR_TEXT(ecvendor_h_kufferath,                        "H. Kufferath GmbH");
    VENDOR_TEXT(ecvendor_eftec,                              "EFTEC Engineering GmbH");
    VENDOR_TEXT(ecvendor_dfc,                                "DFC Design, s.r.o.");
    VENDOR_TEXT(ecvendor_fukuda,                             "FUKUDA CO., LTD.");
    VENDOR_TEXT(ecvendor_fh_flensburg,                       "Fachhochschule Flensburg");
    VENDOR_TEXT(ecvendor_karlsruher_institut,                "Karlsruher Institut fuer Technologie, IAR, H2T");
    VENDOR_TEXT(ecvendor_burnon,                             "Burnon International Ltd.");
    VENDOR_TEXT(ecvendor_nuova,                              "Nuova Fima S.P.A.");
    VENDOR_TEXT(ecvendor_yuban,                              "Yuban and Co.");
    VENDOR_TEXT(ecvendor_ricoh,                              "Ricoh Industry Co., Ltd.");
    VENDOR_TEXT(ecvendor_rdc,                                "RDC Semiconductor Co., Ltd.");
    VENDOR_TEXT(ecvendor_setex_schermuly,                    "SETEX Schermuly textile computer GmbH");
    VENDOR_TEXT(ecvendor_elowerk,                            "elowerk GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_ithemba_labs,                       "iThemba Laboratory for Accelerator Based Sciences (iThemba LABS)");
    VENDOR_TEXT(ecvendor_cncsazan,                           "CNCSAZAN");
    VENDOR_TEXT(ecvendor_shiningview,                        "ShiningView Electronic Technology (Shanghai) Co., Ltd.");
    VENDOR_TEXT(ecvendor_mikysek,                            "Mikysek Engineering");
    VENDOR_TEXT(ecvendor_victron,                            "VICTRON TECHNOLOGY CO., LTD.");
    VENDOR_TEXT(ecvendor_michigan,                           "Michigan Scientific Corporation");
    VENDOR_TEXT(ecvendor_adfweb,                             "ADFweb.com s.r.l.");
    VENDOR_TEXT(ecvendor_nortion,                            "Nortion Servo Technology (Beijing) Co., Ltd.");
    VENDOR_TEXT(ecvendor_gigatronik,                         "GIGATRONIK Korln GmbH");
    VENDOR_TEXT(ecvendor_zhejiang_synmot,                    "Zhejiang Synmot Electrical Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_hk_mns,                             "HK-MnS Co., Ltd");
    VENDOR_TEXT(ecvendor_tattile,                            "Tattile S.r.l.");
    VENDOR_TEXT(ecvendor_elfin,                              "ELFIN Pracownia Elektroniki");
    VENDOR_TEXT(ecvendor_bimba,                              "Bimba Manufacturing Company");
    VENDOR_TEXT(ecvendor_winsonic,                           "Winsonic Electronics Co., Ltd.");
    VENDOR_TEXT(ecvendor_dimetix,                            "DIMETIX AG");
    VENDOR_TEXT(ecvendor_genetec,                            "GENETEC CORPORATION");
    VENDOR_TEXT(ecvendor_tianjin_hengxin,                    "Tianjin Hengxin Chuangyuan Science and Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_sfa,                                "SFA Engineering Corp.");
    VENDOR_TEXT(ecvendor_opticon,                            "Opticon Inc.");
    VENDOR_TEXT(ecvendor_npn,                                "NPN Co., Ltd.");
    VENDOR_TEXT(ecvendor_wuhan_maxsine,                      "Wuhan Maxsine Electric Co., Ltd.");
    VENDOR_TEXT(ecvendor_concept_overdrive,                  "Concept Overdrive Inc.");
    VENDOR_TEXT(ecvendor_hetronik,                           "HETRONIK GmbH");
    VENDOR_TEXT(ecvendor_jbt,                                "JBT Corporation");
    VENDOR_TEXT(ecvendor_tu_dresden_eit,                     "Technische Universitaet Dresden, Fakultaet Elektrotechnik und Informationstechnik");
    VENDOR_TEXT(ecvendor_dajo,                               "DAJO Solutions Ltd.");
    VENDOR_TEXT(ecvendor_criterio,                           "Criterion NDT, Inc.");
    VENDOR_TEXT(ecvendor_quanzhou,                           "Quanzhou Sangchuan Electric Equipment Co., Ltd.");
    VENDOR_TEXT(ecvendor_data_tecno,                         "DATA TECNO Co. Ltd.");
    VENDOR_TEXT(ecvendor_rainbow_springs,                    "Rainbow Springs Pvt Ltd.");
    VENDOR_TEXT(ecvendor_renu,                               "Renu Electronics Pvt. Ltd.");
    VENDOR_TEXT(ecvendor_max_planck_institut,                "Max-Planck-Institut fuer biologische Kybernetik; Wahrnehmung, Kognition und Handlung");
    VENDOR_TEXT(ecvendor_intelligent_automation_zhuhai,      "Intelligent Automation Equipment (Zhuhai) Co., Ltd.");
    VENDOR_TEXT(ecvendor_screen_holdings_ltd,                "SCREEN Holdings Co., Ltd.");
    VENDOR_TEXT(ecvendor_sysmex,                             "Sysmex Corporation");
    VENDOR_TEXT(ecvendor_asm_jp,                             "ASM Japan K.K.");
    VENDOR_TEXT(ecvendor_imago,                              "IMAGO Technologies GmbH");
    VENDOR_TEXT(ecvendor_happiest_minds,                     "Happiest Minds Technologies Private Limited");
    VENDOR_TEXT(ecvendor_open_control_system,                "Open Control System Technology Co. Ltd.");
    VENDOR_TEXT(ecvendor_seoul_uni_engg,                     "University of Seoul, College of Engineering");
    VENDOR_TEXT(ecvendor_ulvac,                              "ULVAC, Inc.");
    VENDOR_TEXT(ecvendor_meliora,                            "Meliora Scientific Inc.");
    VENDOR_TEXT(ecvendor_toshiba_corp,                       "Toshiba Corporation");
    VENDOR_TEXT(ecvendor_schnell,                            "Schnell Spa");
    VENDOR_TEXT(ecvendor_itiri,                              "Industrial Technology Research Institute (ITRI)");
    VENDOR_TEXT(ecvendor_4pico,                              "4PICO BV");
    VENDOR_TEXT(ecvendor_set,                                "SET Power Systems GmbH");
    VENDOR_TEXT(ecvendor_changnam,                           "CHANGNAM I.N.T. LTD.");
    VENDOR_TEXT(ecvendor_shanghai_ruking,                    "Shanghai Ruking Electronic and Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_meta,                               "META Srl");
    VENDOR_TEXT(ecvendor_ghm,                                "GHM Messtechnik GmbH");
    VENDOR_TEXT(ecvendor_xihua_university,                   "Xihua University");
    VENDOR_TEXT(ecvendor_otsl,                               "OTSL Inc.");
    VENDOR_TEXT(ecvendor_aromasoft,                          "Aromasoft Corp.");
    VENDOR_TEXT(ecvendor_bobst,                              "Bobst S.A");
    VENDOR_TEXT(ecvendor_tessera,                            "TESSERA TECHNOLOGY INC.");
    VENDOR_TEXT(ecvendor_jwiesemann,                         "JWiesemann.com - Dr. Joachim Wiesemann");
    VENDOR_TEXT(ecvendor_ulasim_san,                         "Istanbul Ulasim San Tic. A.S.");
    VENDOR_TEXT(ecvendor_prophotonix,                        "ProPhotonix (Irl) Ltd.");
    VENDOR_TEXT(ecvendor_datennetze,                         "B.I.N.S.S Datennetze und Gefahrenmeldesysteme GmbH Berlin");
    VENDOR_TEXT(ecvendor_stefan_cel_mare_university,         "University  Stefan cel Mare Suceava, Electrical Engineering and Computer Science");
    VENDOR_TEXT(ecvendor_modrol,                             "Modrol Electric CO., Ltd.");
    VENDOR_TEXT(ecvendor_obs,                                "OBS Korea Co.,Ltd");
    VENDOR_TEXT(ecvendor_jiao_tong_uni_mech,                 "Shanghai Jiao Tong University, School of Mechanical Engineering");
    VENDOR_TEXT(ecvendor_team,                               "TEAM ELECTRONICS GmbH");
    VENDOR_TEXT(ecvendor_southwest_university,               "Southwest University of Science and Technology (SWUST) National University Science Park");
    VENDOR_TEXT(ecvendor_tdk_lambda,                         "TDK-Lambda Americas Inc.");
    VENDOR_TEXT(ecvendor_ta_liang,                           "Ta Liang Technology Co. Ltd.");
    VENDOR_TEXT(ecvendor_ccs,                                "CCS Inc.");
    VENDOR_TEXT(ecvendor_adaptronic,                         "adaptronic Prueftechnik GmbH");
    VENDOR_TEXT(ecvendor_toho_electronics,                   "TOHO Electronics Inc.");
    VENDOR_TEXT(ecvendor_third_eye,                          "Third Eye Technologies LLC");
    VENDOR_TEXT(ecvendor_nikki_denso,                        "Nikki Denso Co., Ltd.");
    VENDOR_TEXT(ecvendor_systematic,                         "Systematic Consulting Group, Inc.");
    VENDOR_TEXT(ecvendor_peritec,                            "PERITEC Corporation");
    VENDOR_TEXT(ecvendor_bachmann,                           "Bachmann Technology GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_bmk,                                "BMK electronic solutions GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_teco,                               "TECO Electric and Machinery Co., Ltd.");
    VENDOR_TEXT(ecvendor_mechtronic_industries,              "Mechtronic Industries Ltd.");
    VENDOR_TEXT(ecvendor_mernok,                             "Mernok Elektronik (Pty) Ltd.");
    VENDOR_TEXT(ecvendor_bremen_university,                  "Universitaet Bremen, Institut fuer elektrische Antriebe, Leistungselektronik und Bauelemente");
    VENDOR_TEXT(ecvendor_hydro_quebec,                       "Hydro-Quebec Research Institute");
    VENDOR_TEXT(ecvendor_gemtec,                             "GEMTEC Laseroptische Systeme GmbH");
    VENDOR_TEXT(ecvendor_sintef_raufoss,                     "SINTEF Raufoss Manufacturing AS");
    VENDOR_TEXT(ecvendor_advanced_manufacturing,             "Advanced Manufacturing Engineering Technologies Inc.");
    VENDOR_TEXT(ecvendor_levitronix,                         "Levitronix GmbH");
    VENDOR_TEXT(ecvendor_carsten_spiess,                     "Dipl.-Ing. Carsten Spiess Softwareentwicklung");
    VENDOR_TEXT(ecvendor_reivax,                             "Reivax S/A Automation e Controle");
    VENDOR_TEXT(ecvendor_lappeenranta_university,            "Lappeenranta University of Technology (LUT), School of Technology, LUT Energy, Electrical Engineering");
    VENDOR_TEXT(ecvendor_totani,                             "TOTANI CORPORATION");
    VENDOR_TEXT(ecvendor_serad,                              "SERAD S.A.S.");
    VENDOR_TEXT(ecvendor_itw_dynatec,                        "ITW Dynatec GmbH");
    VENDOR_TEXT(ecvendor_hs_emden,                           "Hochschule Emden/Leer, Fachbereich Technik");
    VENDOR_TEXT(ecvendor_jfcontrol,                          "JFControl Co., Ltd.");
    VENDOR_TEXT(ecvendor_sael,                               "SAEL srl");
    VENDOR_TEXT(ecvendor_beckman_coulter,                    "Beckman Coulter Biomedical GmbH");
    VENDOR_TEXT(ecvendor_walter,                             "Walter Maschinenbau GmbH");
    VENDOR_TEXT(ecvendor_alterface,                          "Alterface s.a.");
    VENDOR_TEXT(ecvendor_lectronix,                          "Lectronix, Inc.");
    VENDOR_TEXT(ecvendor_hokuyo,                             "HOKUYO AUTOMATIC CO., LTD.");
    VENDOR_TEXT(ecvendor_shanghai_empower,                   "Shanghai Empower Technologies Co., Ltd.");
    VENDOR_TEXT(ecvendor_thyracont,                          "Thyracont Vacuum Instruments GmbH");
    VENDOR_TEXT(ecvendor_omax,                               "OMAX Corporation");
    VENDOR_TEXT(ecvendor_tox_pressotechnik,                  "TOX PRESSOTECHNIK GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_chiao_tung_university,              "National Chiao Tung University, College of Electrical and Computer Engineering, Department of Electrical Engineering");
    VENDOR_TEXT(ecvendor_inspiro,                            "Inspiro BV");
    VENDOR_TEXT(ecvendor_maxcess,                            "Maxcess International");
    VENDOR_TEXT(ecvendor_chell,                              "Chell Instruments Ltd.");
    VENDOR_TEXT(ecvendor_fuji_machine,                       "FUJI MACHINE MFG. CO., LTD.");
    VENDOR_TEXT(ecvendor_nidec_sankyo,                       "NIDEC SANKYO CORPORATION");
    VENDOR_TEXT(ecvendor_shizuoka_oki,                       "Shizuoka Oki Electric Co., Ltd.");
    VENDOR_TEXT(ecvendor_frencken,                           "Frencken America Inc.");
    VENDOR_TEXT(ecvendor_granite,                            "Granite Devices Oy");
    VENDOR_TEXT(ecvendor_agilicom,                           "AGILiCOM SARL");
    VENDOR_TEXT(ecvendor_philips_technologie,                "Philips Technologie GmbH, Photonics Aachen");
    VENDOR_TEXT(ecvendor_kla_tencor,                         "KLA-Tencor Corporation");
    VENDOR_TEXT(ecvendor_meidensha,                          "Meidensha Corporation");
    VENDOR_TEXT(ecvendor_rolls_royce,                        "Rolls-Royce Nuclear Services");
    VENDOR_TEXT(ecvendor_west_bohemia_university,            "University of West Bohemia, Faculty of Applied Sciences");
    VENDOR_TEXT(ecvendor_keri,                               "Korea Electrotechnology Research Institute (KERI)");
    VENDOR_TEXT(ecvendor_mpi,                                "MPI Corporation");
    VENDOR_TEXT(ecvendor_roeders,                            "Rorders GmbH");
    VENDOR_TEXT(ecvendor_melec,                              "Melec Inc.");
    VENDOR_TEXT(ecvendor_mianyang_weibo,                     "Mianyang Weibo Electronic Co., Ltd.");
    VENDOR_TEXT(ecvendor_wuhan_huazhong,                     "Wuhan Huazhong Numerical Control Co., Ltd.");
    VENDOR_TEXT(ecvendor_datalogic,                          "Datalogic Automation S.r.l.");
    VENDOR_TEXT(ecvendor_tri_tek,                            "Tri-Tek Corp.");
    VENDOR_TEXT(ecvendor_yuheng,                             "YUHENG OPTICS CO.,LTD (Changchun)");
    VENDOR_TEXT(ecvendor_eth,                                "ETH-messtechnik gmbh");
    VENDOR_TEXT(ecvendor_jtekt,                              "JTEKT CORPORATION");
    VENDOR_TEXT(ecvendor_ergo,                               "ergo: elektronik GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_engineerdream,                      "Engineerdream Co., Ltd.");
    VENDOR_TEXT(ecvendor_samsung_electronics,                "Samsung Electronics Co. Ltd.");
    VENDOR_TEXT(ecvendor_ksj,                                "KSJ Co. Ltd.");
    VENDOR_TEXT(ecvendor_messer,                             "Messer Cutting Systems GmbH");
    VENDOR_TEXT(ecvendor_krones,                             "Krones AG");
    VENDOR_TEXT(ecvendor_northwestern_university,            "Northwestern Polytechnical University, School of PowerandEnergy, Department of Power Control and Test");
    VENDOR_TEXT(ecvendor_blackbird,                          "Blackbird Robotersysteme GmbH");
    VENDOR_TEXT(ecvendor_mitsuba,                            "Mitsuba Corporation");
    VENDOR_TEXT(ecvendor_foshan_shunde,                      "Foshan Shunde Gatherwin Information Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_men_mikro,                          "MEN Mikro Elektronik GmbH");
    VENDOR_TEXT(ecvendor_thermo_fisher,                      "Thermo Fisher Scientific Oy");
    VENDOR_TEXT(ecvendor_pyramid,                            "Pyramid Technical Consultants");
    VENDOR_TEXT(ecvendor_verity,                             "Verity Instruments, Inc.");
    VENDOR_TEXT(ecvendor_kai_plus,                           "KAI PLUS TECHNOLOGY CO., LTD.");
    VENDOR_TEXT(ecvendor_texas_a_and_m,                      "Texas AandM University at Qatar, Electrical and Computer Engineering");
    VENDOR_TEXT(ecvendor_fpt,                                "FPT Motorenforschung AG");
    VENDOR_TEXT(ecvendor_machinex,                           "Industries Machinex Inc.");
    VENDOR_TEXT(ecvendor_shanghai_rising_digital,            "Shanghai Rising Digital Co.,Ltd.");
    VENDOR_TEXT(ecvendor_sick_optex,                         "SICK OPTEX CO., LTD.");
    VENDOR_TEXT(ecvendor_laserline,                          "Laserline GmbH");
    VENDOR_TEXT(ecvendor_xpress,                             "Xpress Precision Engineering B.V.");
    VENDOR_TEXT(ecvendor_electra,                            "ELECTRA S.p.A.");
    VENDOR_TEXT(ecvendor_magnescale,                         "Magnescale Co., Ltd.");
    VENDOR_TEXT(ecvendor_hitachi_kokusai,                    "Hitachi Kokusai Electric Inc.");
    VENDOR_TEXT(ecvendor_hangzhou_jingwei,                   "Hangzhou Jingwei Automation Co., Ltd.");
    VENDOR_TEXT(ecvendor_shanghai_lynuc,                     "Shanghai LYNUC CNC Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_atla,                               "ATLAS ELEKTRONIK GmbH");
    VENDOR_TEXT(ecvendor_edac_hangzhou,                      "EDAC Electronics Technology (Hangzhou) Co., Ltd.");
    VENDOR_TEXT(ecvendor_mycom,                              "MYCOM, INC.");
    VENDOR_TEXT(ecvendor_foxconn,                            "Foxconn Technology Group");
    VENDOR_TEXT(ecvendor_qinhuangdao,                        "Qinhuangdao Boostsolar Photovoltatic Equipment Co.,Ltd.");
    VENDOR_TEXT(ecvendor_sia_china,                          "Chinese Academy of Sciences, Shenyang Institute of Automation (SIA)");
    VENDOR_TEXT(ecvendor_deere,                              "Deere and Company");
    VENDOR_TEXT(ecvendor_leibniz_university,                 "Leibniz Universitaet Hannover, Institut fuer Mechatronische Systeme (IMES)ELTRO Gesellschaft fuer Elektrotechnik mbH");
    VENDOR_TEXT(ecvendor_eltro,                              "ELTRO Gesellschaft fuer Elektrotechnik mbH");
    VENDOR_TEXT(ecvendor_uk_grid,                            "UK Grid Solutions Limited");
    VENDOR_TEXT(ecvendor_control_gaging,                     "Control Gaging, Inc.");
    VENDOR_TEXT(ecvendor_hfe,                                "HFE professionelle Studiotechnik GmbH");
    VENDOR_TEXT(ecvendor_schnier,                            "SCHNIER Elektrostatik GmbH");
    VENDOR_TEXT(ecvendor_wuhan_huagong,                      "Wuhan HuaGong Laser Engineering Co.,Ltd.");
    VENDOR_TEXT(ecvendor_zhejiang_keqiang,                   "Zhejiang Keqiang Intelligent Control System Co., Ltd.");
    VENDOR_TEXT(ecvendor_impedans,                           "Impedans Ltd.");
    VENDOR_TEXT(ecvendor_chinese_academy,                    "Chinese Academy of Sciences, Institute of Automation");
    VENDOR_TEXT(ecvendor_gccalliance,                        "GCCAlliance Inc.");
    VENDOR_TEXT(ecvendor_interface,                          "Interface Corporation");
    VENDOR_TEXT(ecvendor_nec,                                "NEC Engineering, Ltd.");
    VENDOR_TEXT(ecvendor_shenyang,                           "Shenyang Piotech Co., Ltd.");
    VENDOR_TEXT(ecvendor_tq_systems,                         "TQ-Systems GmbH");
    VENDOR_TEXT(ecvendor_shanghai_panelmate,                 "Shanghai Panelmate Electronics Co., Ltd.");
    VENDOR_TEXT(ecvendor_haute_ecole_arc,                    "Haute Ecole Arc Ingenierie");
    VENDOR_TEXT(ecvendor_kitech_korea,                       "Korea Institute of Industrial Technology - KITECH");
    VENDOR_TEXT(ecvendor_ceta,                               "CETA Testsysteme GmbH");
    VENDOR_TEXT(ecvendor_step,                               "STEP Corporation");
    VENDOR_TEXT(ecvendor_dalian_guangyang,                   "Dalian Guangyang Science and Technology Group Co., Ltd.");
    VENDOR_TEXT(ecvendor_hermes,                             "Hermes Microvision, Inc.");
    VENDOR_TEXT(ecvendor_kawasaki_heavy,                     "Kawasaki Heavy Industries, Ltd., Robot Division");
    VENDOR_TEXT(ecvendor_elco,                               "ELCO Industry Automation AG");
    VENDOR_TEXT(ecvendor_neuromeka,                          "Neuromeka");
    VENDOR_TEXT(ecvendor_semes,                              "SEMES Co., Ltd.");
    VENDOR_TEXT(ecvendor_d2t,                                "D2T SA");
    VENDOR_TEXT(ecvendor_plasmart,                           "Plasmart Inc.");
    VENDOR_TEXT(ecvendor_drescher,                           "DRESCHER Industrieelektronik GmbH");
    VENDOR_TEXT(ecvendor_mk_system,                          "MK SYSTEM CO.,LTD");
    VENDOR_TEXT(ecvendor_bebro,                              "bebro electronic GmbH");
    VENDOR_TEXT(ecvendor_mc_monitoring,                      "MC-monitoring SA");
    VENDOR_TEXT(ecvendor_variable,                           "Variable Message Signs");
    VENDOR_TEXT(ecvendor_dukane,                             "Dukane Corporation - Intelligent Assembly Solutions");
    VENDOR_TEXT(ecvendor_mecatronix,                         "Mecatronix AG");
    VENDOR_TEXT(ecvendor_prima_power,                        "Prima Power Laserdyne LLC");
    VENDOR_TEXT(ecvendor_isocomp,                            "ISOCOMP srl");
    VENDOR_TEXT(ecvendor_shinko,                             "Shinko Shoji Co.,Ltd.");
    VENDOR_TEXT(ecvendor_asec,                               "ASEC International Corporation");
    VENDOR_TEXT(ecvendor_rtc,                                "RTC Electronics Ltd.");
    VENDOR_TEXT(ecvendor_entegris,                           "Entegris, Inc.");
    VENDOR_TEXT(ecvendor_asem,                               "ASEM S.p.A.");
    VENDOR_TEXT(ecvendor_agie_charmilles,                    "Beijing Agie Charmilles Industrial Electronics Co., Ltd.");
    VENDOR_TEXT(ecvendor_zhengzhou_changhe,                  "Zhengzhou Changhe Electronic Engineering Co., Ltd.");
    VENDOR_TEXT(ecvendor_leister,                            "Leister Technologies AG");
    VENDOR_TEXT(ecvendor_saginomiya,                         "SAGINOMIYA SEISAKUSHO, INC.");
    VENDOR_TEXT(ecvendor_advantech_lnc,                      "Advantech-LNC Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_guangdong_elesy,                    "Guangdong ELESY Electric CO., LTD.");
    VENDOR_TEXT(ecvendor_shanghai_inrevium,                  "SHANGHAI inrevium SOLUTIONS LIMITED");
    VENDOR_TEXT(ecvendor_toyo,                               "TOYO AUTOMATION CO., LTD.");
    VENDOR_TEXT(ecvendor_de_bretagne_sud_university,         "Universite de Bretagne-Sud");
    VENDOR_TEXT(ecvendor_shenzhen_zhiyou,                    "Shenzhen Zhiyou Battery Integration Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_malema,                             "Malema Engineering Corporation");
    VENDOR_TEXT(ecvendor_ricoh_industrial,                   "Ricoh Industrial Solutions Inc.");
    VENDOR_TEXT(ecvendor_tri_city,                           "Tri-City X-ray, LLC");
    VENDOR_TEXT(ecvendor_rutronik,                           "Rutronik Elektronische Bauelemente GmbH");
    VENDOR_TEXT(ecvendor_sanei_hytechs,                      "SANEI HYTECHS VIETNAM Co.,Ltd.");
    VENDOR_TEXT(ecvendor_delixi,                             "Delixi (Hangzhou) Inverter Co.,LTD.");
    VENDOR_TEXT(ecvendor_ritz,                               "RITZ Co., Ltd.");
    VENDOR_TEXT(ecvendor_ricoh_company,                      "Ricoh Company, Ltd.");
    VENDOR_TEXT(ecvendor_tae,                                "TAE Antriebstechnik GmbH");
    VENDOR_TEXT(ecvendor_fontys,                             "Fontys University of Applied Sciences");
    VENDOR_TEXT(ecvendor_hangzhou_riding,                    "Hangzhou Riding Control Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_atlas_copco,                        "Atlas Copco Industrial Technique AB");
    VENDOR_TEXT(ecvendor_mindtribe,                          "Mindtribe Product Engineering, Inc.");
    VENDOR_TEXT(ecvendor_criq,                               "Centre de recherche industrielle du Quebec (CRIQ)");
    VENDOR_TEXT(ecvendor_elster,                             "Elster GmbH");
    VENDOR_TEXT(ecvendor_panasonic_idst,                     "Panasonic Industrial Devices Systems and Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_stv,                                "STV Electronic GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_hentschel,                          "Hentschel System GmbH");
    VENDOR_TEXT(ecvendor_gree,                               "Gree Electric Appliances, Inc. of Zhuhai");
    VENDOR_TEXT(ecvendor_futurestar,                         "Futurestar Corp.");
    VENDOR_TEXT(ecvendor_para_ent,                           "PARA-ENT CO.,LTD.");
    VENDOR_TEXT(ecvendor_siasun,                             "SIASUN CO., LTD.");
    VENDOR_TEXT(ecvendor_wfe,                                "WFE Technology Corporation");
    VENDOR_TEXT(ecvendor_drivexpert,                         "driveXpert GmbH");
    VENDOR_TEXT(ecvendor_luebeck_university,                 "Universitaet zu Luebeck, Institut fuer Medizinische Elektrotechnik");
    VENDOR_TEXT(ecvendor_branson,                            "Branson Ultrasonics Corporation");
    VENDOR_TEXT(ecvendor_rolls_royce_ntulab,                 "Rolls-Royce@NTU Corporate Lab");
    VENDOR_TEXT(ecvendor_guilin_stars,                       "Guilin Stars Science and Technology CO., LTD.");
    VENDOR_TEXT(ecvendor_ace_designers,                      "Ace Designers Limited");
    VENDOR_TEXT(ecvendor_varian,                             "Varian Medical Systems Inc.");
    VENDOR_TEXT(ecvendor_damedics,                           "DAMEDICS GmbH");
    VENDOR_TEXT(ecvendor_energopromavtomatizaciya,           "EnergopromAvtomatizaciya LLC");
    VENDOR_TEXT(ecvendor_microsure,                          "MicroSure B.V.");
    VENDOR_TEXT(ecvendor_finisar,                            "Finisar SHG Inc.");
    VENDOR_TEXT(ecvendor_planet_technology,                  "PLANET Technology Corporation");
    VENDOR_TEXT(ecvendor_pees_components,                    "PEES Components GmbH");
    VENDOR_TEXT(ecvendor_lumberg,                            "Lumberg Automation , a Belden brand");
    VENDOR_TEXT(ecvendor_nachi_fujikoshi,                    "NACHI-FUJIKOSHI CORP.");
    VENDOR_TEXT(ecvendor_radic,                              "Radic Technologies, Inc.");
    VENDOR_TEXT(ecvendor_weightpack,                         "Weightpack S.r.l.");
    VENDOR_TEXT(ecvendor_bs2,                                "BS2 MULTIDATA GmbH");
    VENDOR_TEXT(ecvendor_sumitomo,                           "Sumitomo Heavy Industries, Ltd.");
    VENDOR_TEXT(ecvendor_spectra_physics,                    "Micro-Controle Spectra-Physics S.A.");
    VENDOR_TEXT(ecvendor_apptronik,                          "Apptronik Inc.");
    VENDOR_TEXT(ecvendor_s_haussmann,                        "Dr.-Ing. S. Haussmann Industrieelektronik");
    VENDOR_TEXT(ecvendor_great_river,                        "Great River Electronics, Inc.");
    VENDOR_TEXT(ecvendor_eltra,                              "Eltra S.p.a. Unipersonale");
    VENDOR_TEXT(ecvendor_sinobonder,                         "SINOBONDER Co., Ltd.");
    VENDOR_TEXT(ecvendor_robotous,                           "ROBOTOUS Co., Ltd.");
    VENDOR_TEXT(ecvendor_tianjin_sentinel,                   "Tianjin Sentinel Electronics Co.,Ltd.");
    VENDOR_TEXT(ecvendor_izovac,                             "IZOVAC LTD");
    VENDOR_TEXT(ecvendor_kosice_university,                  "Technical University of Kosice, Faculty of Electrical Engineering and Informatics");
    VENDOR_TEXT(ecvendor_sanmei,                             "SANMEI ELECTRONICS Co., Ltd.");
    VENDOR_TEXT(ecvendor_ea_elektro,                         "EA Elektro-Automatik GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_dynamic_motion,                     "Dynamic Motion Italia S.r.l.");
    VENDOR_TEXT(ecvendor_ooo_pkf,                            "OOO PKF");
    VENDOR_TEXT(ecvendor_shanghai_maihong,                   "SHANGHAI MAIHONG ELECTRONIC TECHNOLOGY CO.LTD");
    VENDOR_TEXT(ecvendor_china_harvest,                      "China Electronics Harvest Technology Co.,Ltd.");
    VENDOR_TEXT(ecvendor_astem_kyoto,                        "Advanced Scientific Technology and Management Research Institute of Kyoto (ASTEM RI)");
    VENDOR_TEXT(ecvendor_mini_motor,                         "Mini Motor srl");
    VENDOR_TEXT(ecvendor_bitifeye,                           "BitifEye Digital Test Solutions GmbH");
    VENDOR_TEXT(ecvendor_ibis,                               "IBIS Computer Pty Ltd");
    VENDOR_TEXT(ecvendor_hanbit,                             "Hanbit Micro Inc.");
    VENDOR_TEXT(ecvendor_tateyama_kagaku,                    "TATEYAMA KAGAKU MODULE TECHNOLOGY CO., LTD.");
    VENDOR_TEXT(ecvendor_aone,                               "Aone Co.,Ltd");
    VENDOR_TEXT(ecvendor_shanghai_cnc,                       "Shanghai Capital Numerical Control Co., Ltd.");
    VENDOR_TEXT(ecvendor_bose,                               "Bose Corporation");
    VENDOR_TEXT(ecvendor_flow,                               "Flow Devices and Systems, Inc.");
    VENDOR_TEXT(ecvendor_anritsu,                            "Anritsu Engineering Co., Ltd.");
    VENDOR_TEXT(ecvendor_emerson_solahd,                     "Emerson SolaHD (a division of Appleton GRP LLC dba Appleton Group)");
    VENDOR_TEXT(ecvendor_modusoft,                           "modusoft GmbH");
    VENDOR_TEXT(ecvendor_sichuan,                            "Sichuan MK Servo Technology");
    VENDOR_TEXT(ecvendor_intron,                             "Intron Technology (China) Co. Ltd.");
    VENDOR_TEXT(ecvendor_jimei_university,                   "Jimei University, College of Information Engineering");
    VENDOR_TEXT(ecvendor_core,                               "CORE CORPORATION");
    VENDOR_TEXT(ecvendor_hib,                                "H.I.B Systemtechnik GmbH");
    VENDOR_TEXT(ecvendor_nova_fabrica,                       "Nova Fabrica Ltd.");
    VENDOR_TEXT(ecvendor_rota,                               "Rota Teknik");
    VENDOR_TEXT(ecvendor_shenzhen_vmmore,                    "SHENZHEN VMMORE CTRLandTECH CO., LTD");
    VENDOR_TEXT(ecvendor_leibniz_uni_eit,                    "Leibniz Universitaet Hannover, Fakultaet fuer Elektrotechnik und Informatik");
    VENDOR_TEXT(ecvendor_saft,                               "Saft S.A.S.");
    VENDOR_TEXT(ecvendor_star_denshi,                        "Star Denshi Co.,Ltd.");
    VENDOR_TEXT(ecvendor_marposs,                            "MARPOSS S.p.A.");
    VENDOR_TEXT(ecvendor_china_orient,                       "China Orient Institute of Noise and Vibration");
    VENDOR_TEXT(ecvendor_cosys,                              "Cosys Inc.");
    VENDOR_TEXT(ecvendor_shenzhen_vector,                    "Shenzhen Vector Automation Technology Co., Lt");
    VENDOR_TEXT(ecvendor_intravis,                           "INTRAVIS GmbH");
    VENDOR_TEXT(ecvendor_drobak,                             "Drobak Unlimited Co.");
    VENDOR_TEXT(ecvendor_georg_simon_ohm,                    "Technische Hochschule Nuernberg Georg Simon Ohm");
    VENDOR_TEXT(ecvendor_zenitron,                           "Zenitron Corporation");
    VENDOR_TEXT(ecvendor_wuhan_endeavor,                     "Wuhan Endeavor Intelligent Machine Co., Ltd.");
    VENDOR_TEXT(ecvendor_leadjeck,                           "LEADJECK AUTOMATION CO., LTD.");
    VENDOR_TEXT(ecvendor_fujian_raynen,                      "Fujian Raynen Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_demcon,                             "Demcon Advanced Mechatronics B.V.");
    VENDOR_TEXT(ecvendor_serva,                              "serva transport systems GmbH");
    VENDOR_TEXT(ecvendor_shenyang_neusoft,                   "Shenyang Neusoft Medical Systems Co., Ltd.");
    VENDOR_TEXT(ecvendor_ruhr,                               "Ruhr-Universitaet Bochum");
    VENDOR_TEXT(ecvendor_banner,                             "Banner Engineering Corporation");
    VENDOR_TEXT(ecvendor_guangdong_topstar,                  "Guangdong Topstar Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_evinsys,                            "Evinsys LLC");
    VENDOR_TEXT(ecvendor_shenzhen_huacheng,                  "Shenzhen Huacheng Industrial Control Co., Ltd.");
    VENDOR_TEXT(ecvendor_mondragon,                          "Mondragon Unibertsitatea");
    VENDOR_TEXT(ecvendor_katholieke,                         "Katholieke Hogeschool Vives (VIVES)");
    VENDOR_TEXT(ecvendor_ekso_bionics,                       "Ekso Bionics Inc.");
    VENDOR_TEXT(ecvendor_kaufman_und_robinson,               "Kaufman and Robinson Inc.");
    VENDOR_TEXT(ecvendor_ksm_electronic,                     "KSM-ELECTRONIC GmbH");
    VENDOR_TEXT(ecvendor_technical_and_try,                  "Technical and Try Co.,Ltd");
    VENDOR_TEXT(ecvendor_maxcom,                             "MAXCOM Co.,Ltd.");
    VENDOR_TEXT(ecvendor_national,                           "National NC System Engineering Research Center");
    VENDOR_TEXT(ecvendor_ryoei,                              "Ryoei Technica Corporation");
    VENDOR_TEXT(ecvendor_schaeffler_technologies,            "Schaeffler Technologies AG and Co. KG");
    VENDOR_TEXT(ecvendor_nts_group,                          "NTS-Group");
    VENDOR_TEXT(ecvendor_tekt,                               "Tekt Industries Pty. Ltd.");
    VENDOR_TEXT(ecvendor_xian_aerospace_automation,          "XiAN Aerospace Automation Co., Ltd");
    VENDOR_TEXT(ecvendor_gal,                                "Gal");
    VENDOR_TEXT(ecvendor_toa,                                "TOA Electronics Inc. Hamatou Company");
    VENDOR_TEXT(ecvendor_friedrich_alexander_university,     "Friedrich-Alexander-Universitaet Erlangen-Nuernberg, Technische Fakultaet");
    VENDOR_TEXT(ecvendor_sirius,                             "Sirius Electronic Systems s.r.l.");
    VENDOR_TEXT(ecvendor_chengdu_inplus,                     "Chengdu InPlus Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_microstep,                          "MicroStep spol s.r.o.");
    VENDOR_TEXT(ecvendor_murata,                             "Murata Machinery, Ltd.");
    VENDOR_TEXT(ecvendor_cabinplant,                         "Cabinplant A/S");
    VENDOR_TEXT(ecvendor_franka_emika,                       "FRANKA EMIKA GmbH");
    VENDOR_TEXT(ecvendor_smart_move,                         "Smart Move GmbH");
    VENDOR_TEXT(ecvendor_ampere,                             "Ampere Inc.");
    VENDOR_TEXT(ecvendor_imkon_endustriyel,                  "Imkon Endustriyel Otomasyon Sistemleri");
    VENDOR_TEXT(ecvendor_tangshan,                           "Tangshan Baichuan Intelligent Machine Co Ltd.");
    VENDOR_TEXT(ecvendor_christian_albrechts,                "Christian-Albrechts-Universitaet zu Kiel");
    VENDOR_TEXT(ecvendor_lorraine_university,                "University of Lorraine, IUT Nancy-Brabois");
    VENDOR_TEXT(ecvendor_fraunhofer_institut_optronik,       "Fraunhofer-Institut fuer Optronik, Systemtechnik und Bildauswertung IOSB");
    VENDOR_TEXT(ecvendor_ip_automatika,                      "IP-Automatika Kft.");
    VENDOR_TEXT(ecvendor_emko,                               "EMKO Elektronik San. ve Tic. A.S.");
    VENDOR_TEXT(ecvendor_fine,                               "FINE Inc.");
    VENDOR_TEXT(ecvendor_uk_atc,                             "Science and Technology Facilities Council, UK Astronomy Technology Centre (UK ATC)");
    VENDOR_TEXT(ecvendor_itmems,                             "ITmems s.r.l.");
    VENDOR_TEXT(ecvendor_lumasense_inc,                      "LumaSense Technologies, Inc.");
    VENDOR_TEXT(ecvendor_acutronic,                          "ACUTRONIC Switzerland Ltd.");
    VENDOR_TEXT(ecvendor_diebie,                             "DieBie EngineeringTSC");
    VENDOR_TEXT(ecvendor_procept,                            "Procept Pty Ltd");
    VENDOR_TEXT(ecvendor_new_power,                          "New Power Plasma Co., Ltd");
    VENDOR_TEXT(ecvendor_amtc,                               "Advanced Mining Technology Center (AMTC)");
    VENDOR_TEXT(ecvendor_asm_singapore,                      "ASM Technology Singapore Pte Ltd.");
    VENDOR_TEXT(ecvendor_weigl,                              "Weigl GmbH and Co KG");
    VENDOR_TEXT(ecvendor_wagner,                             "Wagner International AG");
    VENDOR_TEXT(ecvendor_liebherr,                           "Liebherr-Components Biberach GmbH");
    VENDOR_TEXT(ecvendor_mechatronic,                        "Mechatronics Labs S.r.l.");
    VENDOR_TEXT(ecvendor_jiangsu_torsung,                    "JIANGSU TORSUNG MandE CO.,LTD");
    VENDOR_TEXT(ecvendor_sluzby_bahoza,                      "Technicke sluzby BAHOZA s.r.o.");
    VENDOR_TEXT(ecvendor_itr_institute,                      "Instytut Tele- i Radiotechniczny (ITR)");
    VENDOR_TEXT(ecvendor_beijing_bit,                        "Beijing Institute of Technology (BIT), School of Mechatronical Engineering");
    VENDOR_TEXT(ecvendor_elastisense,                        "ElastiSense ApS");
    VENDOR_TEXT(ecvendor_north_china_university,             "North China University of Technology, Beijing Key Laboratory of Fieldbus and Automation");
    VENDOR_TEXT(ecvendor_nidec,                              "Nidec Corporation");
    VENDOR_TEXT(ecvendor_interface_devices,                  "Interface Devices Ltd.");
    VENDOR_TEXT(ecvendor_nikon,                              "Nikon Corporation");
    VENDOR_TEXT(ecvendor_fujitsu_component,                  "FUJITSU COMPONENT LIMITED");
    VENDOR_TEXT(ecvendor_jiaxing_dealour,                    "Jiaxing Dealour Electric Technology Co.,Ltd.");
    VENDOR_TEXT(ecvendor_eth_zuerich_iris,                   "ETH Zuerich, Department of Mechanical and Process Engineering (D-MAVT), Institute of Robotics and Intelligent Systems (IRIS), Robotic Systems Lab (RSL)");
    VENDOR_TEXT(ecvendor_suruga,                             "SURUGA Production Platform Co., Ltd.");
    VENDOR_TEXT(ecvendor_advanio,                            "Advanio Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_embl_hamburg,                       "EMBL Hamburg");
    VENDOR_TEXT(ecvendor_suruga_seiki,                       "SURUGA SEIKI CO., LTD.");
    VENDOR_TEXT(ecvendor_eva,                                "EVA Robotics Pty Ltd");
    VENDOR_TEXT(ecvendor_beijing_etechwin,                   "Beijing Etechwin Electric Co., Ltd.");
    VENDOR_TEXT(ecvendor_ke_elektronik,                      "KE Elektronik GmbH");
    VENDOR_TEXT(ecvendor_accrea_bartlomiej,                  "ACCREA Bartlomiej Stanczyk");
    VENDOR_TEXT(ecvendor_schunk,                             "SCHUNK GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_sciaky,                             "Sciaky, Inc.");
    VENDOR_TEXT(ecvendor_tokyo_robotics,                     "Tokyo Robotics Inc.");
    VENDOR_TEXT(ecvendor_embeddeers_g,                       "embeddeers GmbH");
    VENDOR_TEXT(ecvendor_gamaco,                             "GAMACO s.r.l.");
    VENDOR_TEXT(ecvendor_npp,                                "NPP VIUS, LLC");
    VENDOR_TEXT(ecvendor_fuji_electronics,                   "FUJI ELECTRONICS CO.,LTD.");
    VENDOR_TEXT(ecvendor_shenzhen_mingsu,                    "SHENZHEN MINGSU AUTOMATION EQUIPMENT CO., LTD");
    VENDOR_TEXT(ecvendor_jiangsu_ysphotech,                  "Jiangsu Ysphotech Technology Co., LTD");
    VENDOR_TEXT(ecvendor_beijing_jcz,                        "Beijing JCZ Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_dmg_mori,                           "DMG MORI CO., LTD.");
    VENDOR_TEXT(ecvendor_telsonic,                           "TELSONIC AG");
    VENDOR_TEXT(ecvendor_ncworks,                            "NCWorks");
    VENDOR_TEXT(ecvendor_van_mierlo,                         "Van Mierlo Ingenieursbureau BV");
    VENDOR_TEXT(ecvendor_control_technology,                 "Control Technology Corporation");
    VENDOR_TEXT(ecvendor_teac,                               "TEAC Corporation");
    VENDOR_TEXT(ecvendor_crossworks,                         "Crossworks Inc.");
    VENDOR_TEXT(ecvendor_agility,                            "Agility Robotics");
    VENDOR_TEXT(ecvendor_shenzhen_double,                    "ShenZhen Double CNC Tech Co., Ltd");
    VENDOR_TEXT(ecvendor_tsc,                                "TSC Inc.");
    VENDOR_TEXT(ecvendor_te_connectivity,                    "TE Connectivity Germany GmbH");
    VENDOR_TEXT(ecvendor_shenzhen_yako,                      "Shenzhen YAKO Automation Technology Co.,Ltd");
    VENDOR_TEXT(ecvendor_prettl,                             "PRETTL Electronics India Pvt. Ltd.");
    VENDOR_TEXT(ecvendor_xiamen_kehua,                       "Xiamen Kehua Hengsheng Co., Ltd.");
    VENDOR_TEXT(ecvendor_mayser,                             "Mayser GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_haitian,                            "HAITIAN Plastics Machinery Group Co., Ltd");
    VENDOR_TEXT(ecvendor_waco,                               "Waco Giken Co., Ltd.");
    VENDOR_TEXT(ecvendor_mike_and_weingartner,               "Mike and Weingartner GmbH");
    VENDOR_TEXT(ecvendor_ionicon,                            "Ionicon Analytik Gesellschaft m.b.H.");
    VENDOR_TEXT(ecvendor_inesc_tec,                          "INESC TEC - Instituto de Engenharia de Sistemas e Computadores Tecnologia e Ciencia");
    VENDOR_TEXT(ecvendor_four_automation,                    "4automation");
    VENDOR_TEXT(ecvendor_moog_ind,                           "Moog India Technology Center Pvt Ltd");
    VENDOR_TEXT(ecvendor_capetown_university,                "University of Cape Town, Department of Electrical Engineering");
    VENDOR_TEXT(ecvendor_auris,                              "Auris Surgical Robotics, Inc.");
    VENDOR_TEXT(ecvendor_ilmenau_university,                 "Technische Universitaet Ilmenau, Fakultaet fuer Maschinenbau, Fachgebiet Mechatronik");
    VENDOR_TEXT(ecvendor_hit_special,                        "HIT SPECIAL ROBOT CO.,LTD");
    VENDOR_TEXT(ecvendor_smartdv,                            "SmartDV Technologies India Private Limited");
    VENDOR_TEXT(ecvendor_visitech,                           "Visitech AS");
    VENDOR_TEXT(ecvendor_solartron,                          "Solartron Metrology Ltd.");
    VENDOR_TEXT(ecvendor_nksystem,                           "NKSystem K.K.");
    VENDOR_TEXT(ecvendor_intronix,                           "INTRONIX spol. s.r.o.");
    VENDOR_TEXT(ecvendor_netmodule,                          "NetModule AG");
    VENDOR_TEXT(ecvendor_bz_robot,                           "BZ Robot INC.");
    VENDOR_TEXT(ecvendor_pues,                               "PUES Corporation");
    VENDOR_TEXT(ecvendor_shs,                                "S.H.S. s.r.l");
    VENDOR_TEXT(ecvendor_manter,                             "Manter International B.V.");
    VENDOR_TEXT(ecvendor_delft_university,                   "Delft University of Technology, Faculty of Aerospace Engineering");
    VENDOR_TEXT(ecvendor_gamade,                             "GAMADE s.n.c. di Westfal Michaele and C.");
    VENDOR_TEXT(ecvendor_ima_tec,                            "ima-tec GmbH");
    VENDOR_TEXT(ecvendor_evest,                              "Evest Corporation");
    VENDOR_TEXT(ecvendor_shinko_technos,                     "SHINKO TECHNOS CO.,LTD.");
    VENDOR_TEXT(ecvendor_sichuan_university,                 "Sichuan University, School of Manufacturing Science and Engineering");
    VENDOR_TEXT(ecvendor_flc_zbigniew,                       "FLC Zbigniew Huber");
    VENDOR_TEXT(ecvendor_taizhou_topcut,                     "Taizhou Topcut-Bullmer Mechanical and Electrical Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_shiratech,                          "Shiratech Embedded Ltd.");
    VENDOR_TEXT(ecvendor_eastern_logic,                      "Eastern Logic Inc.");
    VENDOR_TEXT(ecvendor_weber_schraubautomaten,             "WEBER Schraubautomaten GmbH");
    VENDOR_TEXT(ecvendor_power_solution,                     "Power Solution Network");
    VENDOR_TEXT(ecvendor_advanced_thermal_sciences,          "Advanced Thermal Sciences Corporation");
    VENDOR_TEXT(ecvendor_kitz,                               "KITZ SCT Corporation");
    VENDOR_TEXT(ecvendor_motorcon,                           "Motorcon Inc.");
    VENDOR_TEXT(ecvendor_clownfish,                          "clownfish information technology GmbH");
    VENDOR_TEXT(ecvendor_rabe,                               "RABE Engineering");
    VENDOR_TEXT(ecvendor_ghost,                              "Ghost Robotics LLC");
    VENDOR_TEXT(ecvendor_socionext,                          "Socionext Inc.");
    VENDOR_TEXT(ecvendor_shenzhen_best_motion,               "Shenzhen Best Motion Technology Limited");
    VENDOR_TEXT(ecvendor_escad,                              "ESCAD Automation GmbH");
    VENDOR_TEXT(ecvendor_laval_university,                   "Universite Laval");
    VENDOR_TEXT(ecvendor_mbtech,                             "MBtech Group GmbH and Co. KGaA");
    VENDOR_TEXT(ecvendor_ams,                                "AMS - Gesellschaft fuer Automatisierungs- und Mess-Systemtechnik GmbH");
    VENDOR_TEXT(ecvendor_techno_holon,                       "Techno-Holon Corporation");
    VENDOR_TEXT(ecvendor_j_zimmer,                           "J. Zimmer Maschinenbau GmbH");
    VENDOR_TEXT(ecvendor_e_sigma,                            "e.sigma Technology GmbH");
    VENDOR_TEXT(ecvendor_omron_hangzhou,                     "OMRON AUTOMATION SYSTEM (HANGZHOU) CO.,LTD.");
    VENDOR_TEXT(ecvendor_sof_tek,                            "SOF-TEK Integrators, Inc.");
    VENDOR_TEXT(ecvendor_contrinex,                          "Contrinex SA");
    VENDOR_TEXT(ecvendor_sise,                               "SISE SAS");
    VENDOR_TEXT(ecvendor_christ,                             "Christ Electronic Systems GmbH");
    VENDOR_TEXT(ecvendor_hosta_motion,                       "Hosta Motion Control Co., LTD");
    VENDOR_TEXT(ecvendor_spintrol,                           "Spintrol Limited Corp.");
    VENDOR_TEXT(ecvendor_tema,                               "T.E.M.A. spa");
    VENDOR_TEXT(ecvendor_franz_sprenger,                     "Ingenieurbuero Fuer IC-Technologie Franz Sprenger");
    VENDOR_TEXT(ecvendor_opto4l,                             "OPTO4L GmbH");
    VENDOR_TEXT(ecvendor_quest,                              "QuEST Global Services Pte. Ltd.");
    VENDOR_TEXT(ecvendor_zis,                                "ZIS Industrietechnik GmbH");
    VENDOR_TEXT(ecvendor_lovato,                             "LOVATO Electric S.p.A");
    VENDOR_TEXT(ecvendor_e2m,                                "E2M Technologies B.V.");
    VENDOR_TEXT(ecvendor_zefatek,                            "Zefatek Co., Ltd.");
    VENDOR_TEXT(ecvendor_birket,                             "Birket Engineering, Inc.");
    VENDOR_TEXT(ecvendor_jd,                                 "JD Co., Ltd.");
    VENDOR_TEXT(ecvendor_ica,                                "Institut Clement Ader (ICA)");
    VENDOR_TEXT(ecvendor_okano,                              "OKANO CABLE CO., LTD");
    VENDOR_TEXT(ecvendor_shenzhen_new_medical,               "Shenzhen OUR New Medical Technologies Development Co., Ltd.");
    VENDOR_TEXT(ecvendor_j_schmalz,                          "J. Schmalz GmbH");
    VENDOR_TEXT(ecvendor_fives,                              "Fives OTO S.p.a.");
    VENDOR_TEXT(ecvendor_innocontact,                        "INNOCONTACT CO.,LTD.");
    VENDOR_TEXT(ecvendor_silex,                              "silex technology, Inc.");
    VENDOR_TEXT(ecvendor_shinmaywa,                          "ShinMaywa Industries, LTD.");
    VENDOR_TEXT(ecvendor_ib_prozessleittechnik,              "ib prozessleittechnik GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_ejtech,                             "EJTECH Inc.");
    VENDOR_TEXT(ecvendor_shenzhen_hymson,                    "Shenzhen Hymson Laser Technologies Co.,Ltd.");
    VENDOR_TEXT(ecvendor_flashcut,                           "FlashCut CNC");
    VENDOR_TEXT(ecvendor_beijing_ctb,                        "Beijing CTB technology CO., LTD.");
    VENDOR_TEXT(ecvendor_alexander_binzel,                   "Alexander Binzel Schweisstechnik GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_sawamura_denki,                     "SAWAMURA DENKI IND.CO.,LTD.");
    VENDOR_TEXT(ecvendor_robowell,                           "Robowell Korea Co.");
    VENDOR_TEXT(ecvendor_muse,                               "MUSE Robotics Inc.");
    VENDOR_TEXT(ecvendor_shenzhen_wellauto,                  "Shenzhen WELLAUTO Technology CO., LTD.");
    VENDOR_TEXT(ecvendor_aignep,                             "Aignep S.p.A.");
    VENDOR_TEXT(ecvendor_pusan_national_uni,                 "Pusan National University");
    VENDOR_TEXT(ecvendor_kumoh_mach,                         "KUMOH MACH. and ELEC. CO., LTD.");
    VENDOR_TEXT(ecvendor_imp_china,                          "Chinese Academy of Sciences, Institute of Modern Physics (IMP)");
    VENDOR_TEXT(ecvendor_houston,                            "Houston Mechatronics, Inc");
    VENDOR_TEXT(ecvendor_iar_systems,                        "IAR Systems AB");
    VENDOR_TEXT(ecvendor_koshida,                            "KOSHIDA KOREA CORPORATION");
    VENDOR_TEXT(ecvendor_advanced_micro_fab,                 "Advanced Micro-Fabrication Equipment Inc.");
    VENDOR_TEXT(ecvendor_stephanix,                          "STEPHANIX S.A.");
    VENDOR_TEXT(ecvendor_cpi,                                "CPI Technologies, Inc");
    VENDOR_TEXT(ecvendor_harris,                             "Harris Corporation");
    VENDOR_TEXT(ecvendor_suzhou_agioe,                       "Suzhou AGIOE Equipment Co. Ltd.");
    VENDOR_TEXT(ecvendor_optoelectronics,                    "OPTOELECTRONICS CO., LTD.");
    VENDOR_TEXT(ecvendor_abb_swiss,                          "ABB Switzerland Ltd, Semiconductors");
    VENDOR_TEXT(ecvendor_fms_force,                          "FMS Force Measuring Systems AG");
    VENDOR_TEXT(ecvendor_kulicke_and_soffa,                  "Kulicke and Soffa Industries Inc.");
    VENDOR_TEXT(ecvendor_hexmoto,                            "HEXMOTO Controls Pvt. Ltd");
    VENDOR_TEXT(ecvendor_mitsubishi,                         "Mitsubishi Electric Corporation");
    VENDOR_TEXT(ecvendor_think_surgical,                     "Think Surgical, Inc.");
    VENDOR_TEXT(ecvendor_abb_research,                       "ABB AS Corporate Research");
    VENDOR_TEXT(ecvendor_afag,                               "Afag Automation AG");
    VENDOR_TEXT(ecvendor_shanghai_velle,                     "Shanghai Velle Automobile Air Conditioner Co., Ltd.");
    VENDOR_TEXT(ecvendor_asix,                               "ASIX Electronics Corporation");
    VENDOR_TEXT(ecvendor_hanbaek,                            "Hanbaek Tech Co., Ltd.");
    VENDOR_TEXT(ecvendor_bruker_daltonik,                    "Bruker Daltonik GmbH");
    VENDOR_TEXT(ecvendor_bangkok_university,                 "Bangkok University, School of Engineering, Robotics Laboratory");
    VENDOR_TEXT(ecvendor_cordova_integradores,               "Cordova Industrial Integradores S.A. de C.V.");
    VENDOR_TEXT(ecvendor_seoul_uni,                          "Seoul National University");
    VENDOR_TEXT(ecvendor_cea,                                "CEA LIST");
    VENDOR_TEXT(ecvendor_john_deere,                         "John Deere GmbH and Co. KG, John Deere Werk Mannheim");
    VENDOR_TEXT(ecvendor_deif,                               "DEIF A/S");
    VENDOR_TEXT(ecvendor_wipro,                              "Wipro GE Healthcare Private Limited");
    VENDOR_TEXT(ecvendor_dee_draexlmaier,                    "DEE Draexlmaier Elektrik- und Elektroniksysteme GmbH");
    VENDOR_TEXT(ecvendor_digital_feedback,                   "Digital Feedback Technologies Ltd.");
    VENDOR_TEXT(ecvendor_four_technos,                       "Four Technos Co., Ltd.");
    VENDOR_TEXT(ecvendor_ecs_sistemi,                        "E.C.S. Sistemi Elettronici S.p.A");
    VENDOR_TEXT(ecvendor_espotel,                            "Espotel Oy");
    VENDOR_TEXT(ecvendor_motiv,                              "Motiv Robotics, Inc.");
    VENDOR_TEXT(ecvendor_jcast_networks,                     "Jcast Networks Korea, Inc.");
    VENDOR_TEXT(ecvendor_matrox,                             "Matrox Electronic Systems Ltd.");
    VENDOR_TEXT(ecvendor_veltru,                             "VELTRU AG");
    VENDOR_TEXT(ecvendor_toshiba,                            "TOSHIBA MACHINE CO., LTD");
    VENDOR_TEXT(ecvendor_beijing,                            "Beijing BBF Servo Technology Co.,Ltd.");
    VENDOR_TEXT(ecvendor_a_kyung,                            "A-KYUNG Motion Inc.");
    VENDOR_TEXT(ecvendor_mls_lanny,                          "MLS Lanny GmbH");
    VENDOR_TEXT(ecvendor_cantops,                            "CanTops");
    VENDOR_TEXT(ecvendor_ids,                                "IDS GmbH");
    VENDOR_TEXT(ecvendor_ferag,                              "FERAG AG");
    VENDOR_TEXT(ecvendor_profichip,                          "profichip GmbH");
    VENDOR_TEXT(ecvendor_k_tronic,                           "K-TRONIC S.r.l");
    VENDOR_TEXT(ecvendor_east_group,                         "EAST Group Co., Ltd.");
    VENDOR_TEXT(ecvendor_contexa,                            "Contexa SA");
    VENDOR_TEXT(ecvendor_rt_labs,                            "rt-labs AB");
    VENDOR_TEXT(ecvendor_cosmo,                              "Cosmo Instruments Co.,Ltd.");
    VENDOR_TEXT(ecvendor_advantech,                          "Advantech Co., Ltd.");
    VENDOR_TEXT(ecvendor_adlink,                             "ADLINK TECHNOLOGY INC.");
    VENDOR_TEXT(ecvendor_eubus,                              "eubus GmbH");
    VENDOR_TEXT(ecvendor_precise,                            "Precise Automation, Inc.");
    VENDOR_TEXT(ecvendor_unico,                              "Unico Inc.");
    VENDOR_TEXT(ecvendor_ono_sokki,                          "Ono Sokki Co. Ltd.");
    VENDOR_TEXT(ecvendor_helmholtz_zentrum,                  "Helmholtz-Zentrum Dresden-Rossendorf e.V.");
    VENDOR_TEXT(ecvendor_dlr,                                "DLR Deutsches Zentrum fuer Luft- und Raumfahrt eV");
    VENDOR_TEXT(ecvendor_linke_und_ruehe,                    "PLR Prueftechnik Linke and Ruehe GmbH");
    VENDOR_TEXT(ecvendor_hei_canton_de_vaud_reds,            "Haute Ecole d'Ingenierie et de Gestion du Canton de Vaud du Canton de Vaud");
    VENDOR_TEXT(ecvendor_tamagawa_seiki,                     "TAMAGAWA SEIKI CO.,LTD.");
    VENDOR_TEXT(ecvendor_fujian_huafeng,                     "Fujian Huafeng New Materials Co. Ltd.");
    VENDOR_TEXT(ecvendor_hagenuk,                            "Hagenuk Marinekommunikation GmbH");
    VENDOR_TEXT(ecvendor_sacmi_imola,                        "SACMI IMOLA S.C.");
    VENDOR_TEXT(ecvendor_amada,                              "AMADA Co., Ltd");
    VENDOR_TEXT(ecvendor_hanmi,                              "HANMI Semiconductor Co., Ltd.");
    VENDOR_TEXT(ecvendor_laumas,                             "LAUMAS Elettronica s.r.l.");
    VENDOR_TEXT(ecvendor_dina,                               "DINA Elektronik GmbH");
    VENDOR_TEXT(ecvendor_shenzhen_samkoon,                   "Shenzhen Samkoon Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_pushcorp,                           "PushCorp, Inc.");
    VENDOR_TEXT(ecvendor_syn_tek,                            "SYN-TEK Automation Co., Ltd.");
    VENDOR_TEXT(ecvendor_bystorm,                            "BySTORM and CO. srl");
    VENDOR_TEXT(ecvendor_michel_van_de_wiele,                "Michel Van de Wiele NV");
    VENDOR_TEXT(ecvendor_ipetronik,                          "IPETRONIK GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_opal_rt,                            "Opal-RT Technologies, Inc.");
    VENDOR_TEXT(ecvendor_sennheiser,                         "Sennheiser electronic GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_nextec,                             "Nextec Technologies 2001 Ltd.");
    VENDOR_TEXT(ecvendor_ruethi,                             "Ruethi Electronic AG");
    VENDOR_TEXT(ecvendor_bristol_robotics_lab,               "Bristol Robotics Laboratory");
    VENDOR_TEXT(ecvendor_kinova,                             "Kinova Inc.");
    VENDOR_TEXT(ecvendor_synapticon,                         "Synapticon GmbH");
    VENDOR_TEXT(ecvendor_danieli_automation,                 "DANIELI AUTOMATION SPA");
    VENDOR_TEXT(ecvendor_nanjing_sciyon,                     "Nanjing SCIYON Automation Group Co., Ltd.");
    VENDOR_TEXT(ecvendor_eo_technics,                        "EO Technics Co., Ltd.");
    VENDOR_TEXT(ecvendor_mcs_engenharia,                     "MCS Engenharia Ltda.");
    VENDOR_TEXT(ecvendor_rollmann,                           "Rollmann Elektronik");
    VENDOR_TEXT(ecvendor_burster,                            "burster praezisionsmesstechnik gmbh and co kg");
    VENDOR_TEXT(ecvendor_meiji,                              "MEIJI ELECTRIC INDUSTRIES CO., LTD.");
    VENDOR_TEXT(ecvendor_sanlab_yazilm,                      "Sanlab Software Research Development Energy Engineering San. And Tic. Ltd. Sti");
    VENDOR_TEXT(ecvendor_matrixgroup,                        "MatrixGroup (CMS) Pty Ltd");
    VENDOR_TEXT(ecvendor_four_c,                             "4C Electronics Limited");
    VENDOR_TEXT(ecvendor_purpose,                            "PURPOSE CO., LTD");
    VENDOR_TEXT(ecvendor_eprolinktek,                        "eProLinkTek Co., Ltd.");
    VENDOR_TEXT(ecvendor_ubi,                                "UBI, Inc.");
    VENDOR_TEXT(ecvendor_basel_university,                   "University of Basel, Faculty of Medicine");
    VENDOR_TEXT(ecvendor_acontis,                            "acontis technologies GmbH");
    VENDOR_TEXT(ecvendor_leadshine,                          "Leadshine Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_myostat,                            "Myostat Motion Control Inc.");
    VENDOR_TEXT(ecvendor_energy,                             "Energy Support Corporation");
    VENDOR_TEXT(ecvendor_shanghai,                           "Shanghai University");
    VENDOR_TEXT(ecvendor_fukoku_tokai,                       "FUKOKU TOKAI Co., Ltd.");
    VENDOR_TEXT(ecvendor_sandensha,                          "Sandensha Co., Ltd.");
    VENDOR_TEXT(ecvendor_fraba,                              "FRABA N.V.");
    VENDOR_TEXT(ecvendor_generic_robotics,                   "Generic Robotics Limited");
    VENDOR_TEXT(ecvendor_interworks,                         "INTERWORKS Co., Ltd.");
    VENDOR_TEXT(ecvendor_kratzer,                            "Kratzer Automation AG");
    VENDOR_TEXT(ecvendor_sks_control,                        "SKS Control OY");
    VENDOR_TEXT(ecvendor_tsuzuki_denki,                      "TSUZUKI DENKI CO., LTD");
    VENDOR_TEXT(ecvendor_prestosolution,                     "PRESTOSOLUTION Ltd.");
    VENDOR_TEXT(ecvendor_roesch_and_walter,                  "Rorsch and Walter Industrie-Elektronik GmbH");
    VENDOR_TEXT(ecvendor_sohoaid,                            "SOHOAID Corporation");
    VENDOR_TEXT(ecvendor_kristl,                             "Kristl, Seibt and Co GmbH");
    VENDOR_TEXT(ecvendor_maurizio_ferraris,                  "Maurizio Ferraris (dba Studio Ferraris)");
    VENDOR_TEXT(ecvendor_gerber,                             "Gerber Technology Inc.");
    VENDOR_TEXT(ecvendor_kuka,                               "KUKA Roboter GmbH");
    VENDOR_TEXT(ecvendor_shenzhen_huanan,                    "Shenzhen Huanan Numerical Control System Co., Ltd.");
    VENDOR_TEXT(ecvendor_beijing_jingwei_hirain,             "Beijing Jingwei Hirain Technologies Co.,Ltd.");
    VENDOR_TEXT(ecvendor_qingdao_university,                 "Qingdao Technological University, College of Mechanical Engineering");
    VENDOR_TEXT(ecvendor_gsolar_power,                       "Gsolar Power Co., Ltd.");
    VENDOR_TEXT(ecvendor_china_electronics_university,       "University of Electronic Science and Technology of China");
    VENDOR_TEXT(ecvendor_beijing_jingdiao,                   "Beijing Jingdiao Co., Ltd.");
    VENDOR_TEXT(ecvendor_nat,                                "N.A.T. GmbH");
    VENDOR_TEXT(ecvendor_thermal_dynamics,                   "Thermal Dynamics Oy");
    VENDOR_TEXT(ecvendor_mahr,                               "Mahr GmbH");
    VENDOR_TEXT(ecvendor_hyundai_heavy,                      "Hyundai Heavy Industries Co., Ltd.");
    VENDOR_TEXT(ecvendor_toyota_motor,                       "TOYOTA MOTOR CORPORATION");
    VENDOR_TEXT(ecvendor_yunke_intelligent,                  "YunKe Intelligent Servo Control Technology Co.,Ltd.");
    VENDOR_TEXT(ecvendor_mecapion,                           "LS Mecapion Co., Ltd.");
    VENDOR_TEXT(ecvendor_ontec,                              "ONTEC CO., LTD");
    VENDOR_TEXT(ecvendor_hunan_super,                        "HUNAN SUPER INFORMATION CO., LTD");
    VENDOR_TEXT(ecvendor_foxnum,                             "Foxnum Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_kyoei,                              "Kyoei Electronics Co., Ltd.");
    VENDOR_TEXT(ecvendor_sheltronics,                        "Sheltronics Control Systems Private Limited");
    VENDOR_TEXT(ecvendor_nissin,                             "NISSIN SYSTEMS Co., Ltd.");
    VENDOR_TEXT(ecvendor_tait,                               "Tait Towers Manufacturing, LLC");
    VENDOR_TEXT(ecvendor_tec5,                               "tec5 AG");
    VENDOR_TEXT(ecvendor_satyam,                             "Satyam Computer Services Limited");
    VENDOR_TEXT(ecvendor_china_cepri,                        "China Electric Power Research Institute (CEPRI)");
    VENDOR_TEXT(ecvendor_raonwoori,                          "Raonwoori Technology");
    VENDOR_TEXT(ecvendor_brother,                            "Brother Industries Ltd.");
    VENDOR_TEXT(ecvendor_akribis,                            "Akribis Systems Pte Ltd");
    VENDOR_TEXT(ecvendor_shenyang_machine_tool,              "Shenyang Machine Tool (Group) Design Institute Co., Ltd.");
    VENDOR_TEXT(ecvendor_harbin_institute,                   "Harbin Institute of Technology ShenZhen Graduate School");
    VENDOR_TEXT(ecvendor_sigmatek,                           "SIGMATEK GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_aixtron,                            "AIXTRON SE");
    VENDOR_TEXT(ecvendor_zhejiang_university,                "Zhejiang Sci-Tech University, School of Mechanical Engineering and Automation");
    VENDOR_TEXT(ecvendor_shotover,                           "Shotover Camera Systems LP");
    VENDOR_TEXT(ecvendor_saurer,                             "Saurer AG, Zweigniederlassung Arbon");
    VENDOR_TEXT(ecvendor_shenzhen_hans,                      "Shenzhen Han's Motor SandT Co., Ltd.");
    VENDOR_TEXT(ecvendor_soft_servo,                         "Soft Servo Systems, Inc.");
    VENDOR_TEXT(ecvendor_sungrow,                            "Sungrow Power Supply Co., Ltd.");
    VENDOR_TEXT(ecvendor_vie,                                "ViE Technologies Sdn. Bhd.");
    VENDOR_TEXT(ecvendor_amoy_xiamen,                        "Amoy Dynamics (Xiamen) Co., Ltd.");
    VENDOR_TEXT(ecvendor_mdsi,                               "MDSI Ventures LLC");
    VENDOR_TEXT(ecvendor_atse,                               "ATSE. LLC");
    VENDOR_TEXT(ecvendor_muscle,                             "Muscle Corporation");
    VENDOR_TEXT(ecvendor_hiwin,                              "HIWIN MIKROSYSTEM CORP.");
    VENDOR_TEXT(ecvendor_rmit_university,                    "RMIT University, School of Electrical and Computer Engineering");
    VENDOR_TEXT(ecvendor_beijing_juntai,                     "Beijing Juntai Tiancheng Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_triamec,                            "Triamec Motion AG");
    VENDOR_TEXT(ecvendor_topcon_agriculture,                 "Topcon Precision Agriculture");
    VENDOR_TEXT(ecvendor_fraunhofer_ilt_2,                   "Fraunhofer-Institut fuer Lasertechnik (ILT)");
    VENDOR_TEXT(ecvendor_anedo,                              "ANEDO Ltd.");
    VENDOR_TEXT(ecvendor_galaxy_fareast,                     "Galaxy Far East Corp.");
    VENDOR_TEXT(ecvendor_vipa_visualsierung,                 "VIPA Gesellschaft fuer Visualisierung und Prozessautomatisierung mbH");
    VENDOR_TEXT(ecvendor_orsys_orth,                         "Orsys Orth System GmbH");
    VENDOR_TEXT(ecvendor_viveris,                            "Viveris Technologies");
    VENDOR_TEXT(ecvendor_sensa,                              "Sensa automatisering BV");
    VENDOR_TEXT(ecvendor_brainchild,                         "Brainchild Electronic Co., Ltd.");
    VENDOR_TEXT(ecvendor_beta_dyn,                           "BETA Dyn GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_gd,                                 "G.D SpA");
    VENDOR_TEXT(ecvendor_national_radio_astronomy,           "National Radio Astronomy Observatory");
    VENDOR_TEXT(ecvendor_riedl,                              "Riedl GmbH");
    VENDOR_TEXT(ecvendor_cisa,                               "CISA Intelligent Systems and Automation S.L.");
    VENDOR_TEXT(ecvendor_jiangsu_cptek,                      "Jiangsu CPTEK Servo Technology Co.Ltd.");
    VENDOR_TEXT(ecvendor_sotec,                              "SOTEC Software Entwicklungs GmbH + Co. Mikrocomputertechnik KG");
    VENDOR_TEXT(ecvendor_keba_at,                            "KEBA AG");
    VENDOR_TEXT(ecvendor_dabo,                               "DABO Corporation");
    VENDOR_TEXT(ecvendor_dalian_university,                  "Dalian University of Technology");
    VENDOR_TEXT(ecvendor_potomac,                            "Potomac Electric Corporation");
    VENDOR_TEXT(ecvendor_esab,                               "ESAB Welding and Cutting GmbH");
    VENDOR_TEXT(ecvendor_meastream,                          "meastream GmbH");
    VENDOR_TEXT(ecvendor_emmission,                          "Emmission");
    VENDOR_TEXT(ecvendor_willow_garage,                      "Willow Garage, Inc.");
    VENDOR_TEXT(ecvendor_shanghai_passiontech,               "Shanghai Passiontech Information Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_imagos_di_renato,                   "Imagos S.a.s di Renato Andreola e C.");
    VENDOR_TEXT(ecvendor_fab_9,                              "Fab-9 Corporation");
    VENDOR_TEXT(ecvendor_kirp_korea,                         "Korea Institute of Robot and Convergence (KIRO)");
    VENDOR_TEXT(ecvendor_interroll_trommelmotoren,           "Interroll Trommelmotoren GmbH");
    VENDOR_TEXT(ecvendor_silica_avnet,                       "Silica, Avnet EMG GmbH");
    VENDOR_TEXT(ecvendor_harbin_inst_Technology,             "Harbin Institute of Technology");
    VENDOR_TEXT(ecvendor_ueno_seiki,                         "UENO SEIKI Co.,LTD.");
    VENDOR_TEXT(ecvendor_rb3d,                               "RB3D");
    VENDOR_TEXT(ecvendor_robotiq,                            "Robotiq Inc.");
    VENDOR_TEXT(ecvendor_hitachi_terminal,                   "Hitachi Terminal Mechatronics, Corp.");
    VENDOR_TEXT(ecvendor_glidewell,                          "Glidewell Laboratories");
    VENDOR_TEXT(ecvendor_beijing_saintwise,                  "Beijing SaintWise Intelligent Technology Development co.LTD");
    VENDOR_TEXT(ecvendor_beijing_sunwise,                    "Beijing Sunwise Space Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_servotecnica_spa,                   "Servotecnica SpA");
    VENDOR_TEXT(ecvendor_utthunga,                           "Utthunga Technologies Pvt. Ltd.");
    VENDOR_TEXT(ecvendor_beijing_university_posts,           "Beijing University of Posts and Telecommunications, School of Automation");
    VENDOR_TEXT(ecvendor_carl_zeiss_optotechnik,             "Carl Zeiss Optotechnik GmbH");
    VENDOR_TEXT(ecvendor_carl_zeiss_smt,                     "Carl Zeiss SMT GmbH");
    VENDOR_TEXT(ecvendor_sichuan_dongfang,                   "Sichuan Dongfang Electric Autocontrol Engineering Co., Ltd.");
    VENDOR_TEXT(ecvendor_xp_power,                           "XP Power");
    VENDOR_TEXT(ecvendor_sioux,                              "Sioux CCM B.V.");
    VENDOR_TEXT(ecvendor_syswork,                            "SYSWORK CO.,LTD.");
    VENDOR_TEXT(ecvendor_sick_inc,                           "SICK, Inc.");
    VENDOR_TEXT(ecvendor_hunan_matrix,                       "Hunan Matrix Electronic Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_shenzhen_tongchuan,                 "ShenZhen Tongchuan Technology Co.,Ltd");
    VENDOR_TEXT(ecvendor_itk,                                "ITK Engineering GmbH");
    VENDOR_TEXT(ecvendor_alexan,                             "Alexan Tech. Inc.");
    VENDOR_TEXT(ecvendor_nanjing_daqo,                       "Nanjing Daqo Electrical Institute Co., Ltd.");
    VENDOR_TEXT(ecvendor_cornelius,                          "Cornelius Consult");
    VENDOR_TEXT(ecvendor_quality_firmware,                   "Quality Firmware and Processes Solutions, LLC");
    VENDOR_TEXT(ecvendor_yes_energy,                         "YES ENERGY srl");
    VENDOR_TEXT(ecvendor_chroma_ate,                         "Chroma ATE Inc.");
    VENDOR_TEXT(ecvendor_toshiba_transport,                  "Toshiba Transport Engineering Inc.");
    VENDOR_TEXT(ecvendor_opvengineering,                     "OPVengineering GmbH");
    VENDOR_TEXT(ecvendor_shenzhen_inovance,                  "Shenzhen Inovance Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_serwis,                             "SERWIS CNC Mariusz Mareczko");
    VENDOR_TEXT(ecvendor_sbs_science,                        "SBS Science and Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_zhejiang_feida,                     "Zhejiang Feida Environmental Science and Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_robotic_systems,                    "Robotic Systems Integration, Inc.");
    VENDOR_TEXT(ecvendor_youngtek,                           "YoungTek Electronics Corp.");
    VENDOR_TEXT(ecvendor_beijing_mechatronical,              "Beijing Institute of Technology, School of Mechatronical Engineering");
    VENDOR_TEXT(ecvendor_eraetech,                           "ERAETECH Co., Ltd.");
    VENDOR_TEXT(ecvendor_easy_etudes,                        "Easy Etudes et applications systeme SA");
    VENDOR_TEXT(ecvendor_beijing_richauto,                   "Beijing RichAuto SandT Co., Ltd");
    VENDOR_TEXT(ecvendor_ulvac_automation,                   "ULVAC AUTOMATION TAIWAN Inc.");
    VENDOR_TEXT(ecvendor_shenyang_xbang,                     "Shenyang XBANG Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_start_to_sail,                      "Guangzhou Start To Sail Industrial Robot Co.,Ltd.");
    VENDOR_TEXT(ecvendor_wu_yang,                            "WU-YANG Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_smartray,                           "SmartRay GmbH");
    VENDOR_TEXT(ecvendor_ningbo_yunsheng,                    "Ningbo Yunsheng Co., Ltd.");
    VENDOR_TEXT(ecvendor_kayser_threde,                      "Kayser-Threde GmbH");
    VENDOR_TEXT(ecvendor_ege_robotics,                       "Ege Robotics CNC Makine Elektronik Otomasyon Medikal");
    VENDOR_TEXT(ecvendor_tallinn_university,                 "Tallinn University of Technology (TUT), Faculty of Information Technology");
    VENDOR_TEXT(ecvendor_tallinn_university_it,              "Tallinn University of Technology (TUT), Faculty of Information Technology");
    VENDOR_TEXT(ecvendor_meerecompany,                       "meerecompany Inc.");
    VENDOR_TEXT(ecvendor_research_and_production,            "Research and Production Plant EKRA");
    VENDOR_TEXT(ecvendor_fpt_software,                       "FPT Software Ltd.");
    VENDOR_TEXT(ecvendor_emotion,                            "eMotion Co., Ltd.");
    VENDOR_TEXT(ecvendor_hrk_brk,                            "HRK-BRK SRLS");
    VENDOR_TEXT(ecvendor_nanjing_chenguang,                  "Nanjing Chenguang Group Co., Ltd.");
    VENDOR_TEXT(ecvendor_manner,                             "Manner Sensortelemetrie GmbH");
    VENDOR_TEXT(ecvendor_smart_testsolutions,                "SMART TESTSOLUTIONS GmbH");
    VENDOR_TEXT(ecvendor_meinhard_koppitz,                   "Meinhard Koppitz, Elektronikentwicklung");
    VENDOR_TEXT(ecvendor_ruag_defence,                       "RUAG Defence Deutschland GmbH");
    VENDOR_TEXT(ecvendor_creasoft,                           "Creasoft SL");
    VENDOR_TEXT(ecvendor_shanghai_rui_fast,                  "Shanghai Rui Fast Automation Equipment Co.,Ltd.");
    VENDOR_TEXT(ecvendor_egle,                               "Egle Systems S.L.");
    VENDOR_TEXT(ecvendor_viewmove,                           "ViewMove Technologies Inc.");
    VENDOR_TEXT(ecvendor_altima,                             "ALTIMA Corp.");
    VENDOR_TEXT(ecvendor_anybotics,                          "ANYbotics AG");
    VENDOR_TEXT(ecvendor_apl_landau,                         "APL Automobil-Prueftechnik Landau GmbH");
    VENDOR_TEXT(ecvendor_ailux,                              "AiLux S.r.l.");
    VENDOR_TEXT(ecvendor_seren,                              "Seren Industrial Power Systems, Inc.");
    VENDOR_TEXT(ecvendor_cit_chiba,                          "Chiba Institute of Technology (CIT)");
    VENDOR_TEXT(ecvendor_cambridge_micro_engg,               "Cambridge Micro Engineering Limited");
    VENDOR_TEXT(ecvendor_cimon,                              "CIMON CO.,LTD.");
    VENDOR_TEXT(ecvendor_d_and_v,                            "DandV Electronics Ltd.");
    VENDOR_TEXT(ecvendor_distek,                             "DISTek Integration, Inc.");
    VENDOR_TEXT(ecvendor_astrodyne,                          "Astrodyne TDI");
    VENDOR_TEXT(ecvendor_daewoo,                             "Daewoo Shipbuilding and Marine Engineering Co., Ltd.");
    VENDOR_TEXT(ecvendor_electronic_theatre,                 "Electronic Theatre Controls, Inc.");
    VENDOR_TEXT(ecvendor_itmo_university,                    "ITMO University, Department of Control Systems and Industrial Robotics, Chair of Electrical ");
    VENDOR_TEXT(ecvendor_envision_jiangsu,                   "Envision Energy (Jiangsu) CO., LTD.");
    VENDOR_TEXT(ecvendor_future_electronics,                 "Future Electronics Inc.");
    VENDOR_TEXT(ecvendor_ginolis,                            "Ginolis Oy");
    VENDOR_TEXT(ecvendor_innodelec,                          "Innodelec Sarl");
    VENDOR_TEXT(ecvendor_hexagon_metrology,                  "Hexagon Metrology S.p.A.");
    VENDOR_TEXT(ecvendor_headspring,                         "Headspring Inc.");
    VENDOR_TEXT(ecvendor_kuhnke,                             "Kendrion Kuhnke Automation GmbH");
    VENDOR_TEXT(ecvendor_hv_sistemas,                        "HV Sistemas S.L.");
    VENDOR_TEXT(ecvendor_ibv_echtzeit,                       "IBV - Echtzeit- und Embedded GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_icp_das,                            "ICP DAS Co.,Ltd");
    VENDOR_TEXT(ecvendor_irt,                                "IRT SA");
    VENDOR_TEXT(ecvendor_seoul_precision,                    "Seoul Precision Machines Co., Ltd.");
    VENDOR_TEXT(ecvendor_jat,                                "Jenaer Antriebstechnik GmbH");
    VENDOR_TEXT(ecvendor_linz_center,                        "Linz Center of Mechatronics GmbH");
    VENDOR_TEXT(ecvendor_lithoz,                             "Lithoz GmbH");
    VENDOR_TEXT(ecvendor_mas_systec,                         "MAS-SysTec GmbH");
    VENDOR_TEXT(ecvendor_chr_mayr,                           "Chr. Mayr GmbH + Co. KG");
    VENDOR_TEXT(ecvendor_heidelberger,                       "Heidelberger Druckmaschinen AG");
    VENDOR_TEXT(ecvendor_mecalc,                             "Mecalc PTY Limited");
    VENDOR_TEXT(ecvendor_volga_state_university,             "Volga State University of Technology, Faculty of Information Technologies and Computer Engineering");
    VENDOR_TEXT(ecvendor_narae_nanotech,                     "NARAE NANOTECH Corporation");
    VENDOR_TEXT(ecvendor_nemonos,                            "NEMONOS GmbH");
    VENDOR_TEXT(ecvendor_opsens,                             "Opsens Inc.");
    VENDOR_TEXT(ecvendor_ormec_systems,                      "ORMEC Systems Corp.");
    VENDOR_TEXT(ecvendor_posco,                              "POSCO ENGINEERING Co., Ltd.");
    VENDOR_TEXT(ecvendor_pc_krause,                          "PC Krause and Associates, Inc.");
    VENDOR_TEXT(ecvendor_powersparks,                        "PowerSparks GmbH");
    VENDOR_TEXT(ecvendor_paul_scherrer,                      "Paul Scherrer Institut");
    VENDOR_TEXT(ecvendor_packsize,                           "Packsize Technologies AB");
    VENDOR_TEXT(ecvendor_shanghai_volboff,                   "Shanghai VolBoff Electron Science and Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_daegu_gyeongbuk,                    "Daegu Gyeongbuk Institute of Science and Technology, Robotics System Research Division");
    VENDOR_TEXT(ecvendor_samsung_sec,                        "Samsung Electronics Co. Ltd.");
    VENDOR_TEXT(ecvendor_starfire,                           "Starfire Industries, LLC");
    VENDOR_TEXT(ecvendor_sfera,                              "SFERA S.r.l.");
    VENDOR_TEXT(ecvendor_sg_electronic,                      "SG Electronic Systems SRLS");
    VENDOR_TEXT(ecvendor_ssi,                                "SSI CO.,LTD.");
    VENDOR_TEXT(ecvendor_tokyo_electron_ltd,                 "Tokyo Electron Limited");
    VENDOR_TEXT(ecvendor_tem_tech,                           "Tem-Tech Lab.");
    VENDOR_TEXT(ecvendor_tomen,                              "TOMEN ELECTRONICS CORPORATION");
    VENDOR_TEXT(ecvendor_ste_trekwer,                        "STE Trekwerk B.V.");
    VENDOR_TEXT(ecvendor_team_new_zealand,                   "TEAM NEW ZEALAND AC35 CHALLENGE LIMITED (dba Emirates Team New Zealand)");
    VENDOR_TEXT(ecvendor_tronteq_electronic,                 "TRONTEQ Electronic GbR");
    VENDOR_TEXT(ecvendor_techman,                            "Techman Robot Inc.");
    VENDOR_TEXT(ecvendor_sakarya_universityent,              "Sakarya University, Faculty of Computer and Information Sciences, Computer Engineering Department");
    VENDOR_TEXT(ecvendor_js_automation,                      "JS Automation Corp.");
    VENDOR_TEXT(ecvendor_ueidaq,                             "United Electronic Industries, Inc. (UEIDAQ)");
    VENDOR_TEXT(ecvendor_jiangxi_fashion,                    "Jiangxi Fashion Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_shenzhen_v_and_t,                   "ShenZhen VandT Technologies Co., Ltd.");
    VENDOR_TEXT(ecvendor_vmi,                                "VMI Holland B.V.");
    VENDOR_TEXT(ecvendor_weatherford,                        "Weatherford Oil Tool GmbH");
    VENDOR_TEXT(ecvendor_witium,                             "Witium Co., Ltd.");
    VENDOR_TEXT(ecvendor_inca,                               "Inca Digital Printers Limited");
    VENDOR_TEXT(ecvendor_cerebrus,                           "Cerebrus Corporation");
    VENDOR_TEXT(ecvendor_xiamen_micromatch,                  "Xiamen Micromatch Electronic Information Technology Co.,Ltd.");
    VENDOR_TEXT(ecvendor_hangzhou_zhenzheng,                 "Hangzhou Zhenzheng Robot Technology Co.,LTD");
    VENDOR_TEXT(ecvendor_zygo,                               "Zygo Corporation");
    VENDOR_TEXT(ecvendor_google,                             "Google Inc.");
    VENDOR_TEXT(ecvendor_aixcon,                             "aixcon PowerSystems GmbH");
    VENDOR_TEXT(ecvendor_maple,                              "Maple Electronics");
    VENDOR_TEXT(ecvendor_yuchang,                            "YUCHANG TECH Co., Ltd.");
    VENDOR_TEXT(ecvendor_changzhou,                          "Changzhou MVision IT Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_bilko,                              "Bilko Computer Control and Automation Co.");
    VENDOR_TEXT(ecvendor_kinco_electric,                     "Kinco Electric (Shenzhen) Ltd.");
    VENDOR_TEXT(ecvendor_shenzhen_hengketong,                "SHENZHEN HENGKETONG ROBOT CO., LTD");
    VENDOR_TEXT(ecvendor_mianyang_fude,                      "Mianyang Fude Robot Co., Ltd.");
    VENDOR_TEXT(ecvendor_kjellberg_finsterwalde,             "Kjellberg Finsterwalde Plasma und Maschinen GmbH");
    VENDOR_TEXT(ecvendor_shodensha,                          "SHODENSHA Co., Ltd.");
    VENDOR_TEXT(ecvendor_cyber_control,                      "Cyber Control Systems LLC");
    VENDOR_TEXT(ecvendor_ponutech,                           "PONUTech");
    VENDOR_TEXT(ecvendor_mechadept,                          "MechAdept Limited");
    VENDOR_TEXT(ecvendor_bst_eltromat,                       "BST eltromat International GmbH");
    VENDOR_TEXT(ecvendor_rainbow,                            "Rainbow Co.");
    VENDOR_TEXT(ecvendor_sentrol_ltd,                        "SENTROL Co., Ltd.");
    VENDOR_TEXT(ecvendor_hangzhou_dianzi_university,         "HangZhou Dianzi University, School of Mechanical Engineering");
    VENDOR_TEXT(ecvendor_devol_advanced,                     "Devol Advanced Automation, Inc.");
    VENDOR_TEXT(ecvendor_chengdu_yanxing,                    "Chengdu Yanxing Automation Engineering Co., Ltd.");
    VENDOR_TEXT(ecvendor_metis,                              "METIS Co., Ltd.");
    VENDOR_TEXT(ecvendor_goldluecke,                         "Goldluecke GmbH");
    VENDOR_TEXT(ecvendor_microtime,                          "Microtime Computer Inc.");
    VENDOR_TEXT(ecvendor_kist_1,                             "Korea Institute of Science and Technology(KIST)");
    VENDOR_TEXT(ecvendor_kist_2,                             "Korea Institute of Science and Technology (KIST)");
    VENDOR_TEXT(ecvendor_shenzhen_veichi,                    "Shenzhen Veichi Electric Co., Ltd");
    VENDOR_TEXT(ecvendor_atop,                               "Atop Technologies, Inc.");
    VENDOR_TEXT(ecvendor_simplo,                             "Simplo Technology CO., LTD.");
    VENDOR_TEXT(ecvendor_hitachi_hightech,                   "Hitachi High-Technologies Corporation");
    VENDOR_TEXT(ecvendor_dongguan_avatar,                    "Dongguan Avatar System Automation Equipment Co., Ltd.");
    VENDOR_TEXT(ecvendor_hangzhou_wahaha,                    "Hangzhou Wahaha Group Co., LTD., Mechanical and Electrical Institute");
    VENDOR_TEXT(ecvendor_smartind,                           "Smartind Technologies Co., Ltd.");
    VENDOR_TEXT(ecvendor_opsens_soutions,                    "Opsens Solutions Inc.");
    VENDOR_TEXT(ecvendor_shanghai_maritime_university,       "Shanghai Maritime University, Logistics Engineering College");
    VENDOR_TEXT(ecvendor_aselsa,                             "ASELSAN A.Z.");
    VENDOR_TEXT(ecvendor_inventec_shanghai,                  "Inventec Appliances (Shanghai) Co., Ltd.");
    VENDOR_TEXT(ecvendor_bitvis,                             "Bitvis AS");
    VENDOR_TEXT(ecvendor_shenzhen_sipake,                    "Shenzhen Sipake Electric Co., Ltd.");
    VENDOR_TEXT(ecvendor_altera,                             "Altera Corporation");
    VENDOR_TEXT(ecvendor_abatec,                             "abatec group ag");
    VENDOR_TEXT(ecvendor_promotion_comercio,                 "Promotion Comercio e Servico Ltda");
    VENDOR_TEXT(ecvendor_enders,                             "enders Ingenieure GmbH");
    VENDOR_TEXT(ecvendor_hanyang_robotics,                   "HANYANG ROBOTICS CO.,LTD");
    VENDOR_TEXT(ecvendor_analog_devices,                     "Analog Devices, Inc.");
    VENDOR_TEXT(ecvendor_kk_electronic,                      "KK-electronic as");
    VENDOR_TEXT(ecvendor_hofo,                               "HOFO Automation Co., Ltd.");
    VENDOR_TEXT(ecvendor_pal,                                "PAL Robotics S.L.");
    VENDOR_TEXT(ecvendor_moog_animatics,                     "Moog Animatics");
    VENDOR_TEXT(ecvendor_largan,                             "Largan Precision Co.,Ltd.");
    VENDOR_TEXT(ecvendor_china_electronics_21,               "China Electronics Technology Group Corporation, No. 21 Research InstituteREXA Inc.");
    VENDOR_TEXT(ecvendor_rexa,                               "REXA Inc.");
    VENDOR_TEXT(ecvendor_dave,                               "DAVE Srl");
    VENDOR_TEXT(ecvendor_daeati,                             "DAEATI Co., Ltd.");
    VENDOR_TEXT(ecvendor_dain_cube,                          "DAIN CUBE");
    VENDOR_TEXT(ecvendor_shanghai_xpartner,                  "Shanghai xPartner Robotics Co.,Ltd.");
    VENDOR_TEXT(ecvendor_eso_eu,                             "ESO, European Southern Observatory");
    VENDOR_TEXT(ecvendor_weld_tooling,                       "Weld Tooling Corporation (dba BUG-O Systems)");
    VENDOR_TEXT(ecvendor_tongtai,                            "Tongtai Machine and Tool Co., Ltd.");
    VENDOR_TEXT(ecvendor_promax,                             "PROMAX srl");
    VENDOR_TEXT(ecvendor_hitex_uk,                           "Hitex (UK) Ltd.");
    VENDOR_TEXT(ecvendor_mecademic,                          "Mecademic Inc.");
    VENDOR_TEXT(ecvendor_stoegra,                            "STOEGRA Antriebstechnik GmbH");
    VENDOR_TEXT(ecvendor_entec_electric,                     "ENTEC Electric and Electronic CO., LTD.");
    VENDOR_TEXT(ecvendor_henan_college,                      "Henan Mechanical and Electrical Vocational College");
    VENDOR_TEXT(ecvendor_eonyk,                              "Eonyk AG");
    VENDOR_TEXT(ecvendor_williams_grand,                     "Williams Grand Prix Engineering Limited");
    VENDOR_TEXT(ecvendor_koyo,                               "KOYO ELECTRONICS INDUSTRIES CO., LTD.");
    VENDOR_TEXT(ecvendor_gtd_sistemas,                       "GTD Sistemas de Informacian SA");
    VENDOR_TEXT(ecvendor_kemppi,                             "Kemppi Oy");
    VENDOR_TEXT(ecvendor_chongqing_huashu,                   "Chongqing Huashu Robotics Co.,Ltd.");
    VENDOR_TEXT(ecvendor_arlington_lab,                      "Arlington Laboratory Corporation");
    VENDOR_TEXT(ecvendor_sick_ag,                            "SICK AG");
    VENDOR_TEXT(ecvendor_omron_adept,                        "Omron Adept Technologies, Inc");
    VENDOR_TEXT(ecvendor_parker_hannifin_id2,                "Parker Hannifin Manufacturing S.r.l");
    VENDOR_TEXT(ecvendor_balluf,                             "Balluff GmbH");
    VENDOR_TEXT(ecvendor_control_techniques,                 "Control Techniques GmbH");
    VENDOR_TEXT(ecvendor_shanghai_sany_science,              "SHANGHAI SANY SCIENCE and TECHNOLOGY CO., LTD");
    VENDOR_TEXT(ecvendor_infineon_technologies_us,           "Infineon Technologies Americas Corporation");
    VENDOR_TEXT(ecvendor_renesas_korea,                      "Renesas Electronics Korea Co., Ltd.");
    VENDOR_TEXT(ecvendor_bimba_manufacturing,                "Bimba Manufacturing Company");
    VENDOR_TEXT(ecvendor_screen_holdings,                    "SCREEN Holdings Co., Ltd.");
    VENDOR_TEXT(ecvendor_frencken_us,                        "Frencken America Inc.");
    VENDOR_TEXT(ecvendor_kla_tencor_corp,                    "KLA-Tencor Corporation");
    VENDOR_TEXT(ecvendor_edac_electronics_hangzhou,          "EDAC Electronics Technology (Hangzhou) Co., Ltd.");
    VENDOR_TEXT(ecvendor_core_corp,                          "CORE CORPORATION");
    VENDOR_TEXT(ecvendor_drobak_unlimited,                   "Drobak Unlimited Co.");
    VENDOR_TEXT(ecvendor_amada_miyachi,                      "AMADA MIYACHI EUROPE GmbH");
    VENDOR_TEXT(ecvendor_eprolinktek_ltd,                    "eProLinkTek Co., Ltd.");
    VENDOR_TEXT(ecvendor_hyundai_heavy_ltd,                  "Hyundai Heavy Industries Co., Ltd.");
    VENDOR_TEXT(ecvendor_hunan_super_information,            "HUNAN SUPER INFORMATION CO., LTD");
    VENDOR_TEXT(ecvendor_fraunhofer_lasertechnik_ilt,        "Fraunhofer-Institut fuer Lasertechnik (ILT)");
    VENDOR_TEXT(ecvendor_kinco_electric_schenzen,            "Kinco Electric (Shenzhen) Ltd.");
    VENDOR_TEXT(ecvendor_time,                               "TIME GROUP Inc.");
    VENDOR_TEXT(ecvendor_ma_vi,                              "Ma.Vi. srl");
    VENDOR_TEXT(ecvendor_parker_hannifin_eme,                "Parker Hannifin GmbH - EME");
    VENDOR_TEXT(ecvendor_danfoss_drives,                     "Danfoss Drives A/S");
    VENDOR_TEXT(ecvendor_fraunhofer_ilt,                     "Fraunhofer-Institut fuer Lasertechnik (ILT)");
    VENDOR_TEXT(ecvendor_shanghai_sany_sciencetech,          "SHANGHAI SANY SCIENCE and TECHNOLOGY CO., LTD");
    VENDOR_TEXT(ecvendor_infineon_technologies_china,        "Infineon Technologies China Co., Ltd.");
    VENDOR_TEXT(ecvendor_parker_hannifin_eme_630,            "Parker Hannifin GmbH - EME");
    VENDOR_TEXT(ecvendor_danfoss,                            "Danfoss GmbH");
    VENDOR_TEXT(ecvendor_parker_hannifin_ssd,                "Parker Hannifin Ltd.");
    VENDOR_TEXT(ecvendor_longxin_zhijian,                    "Longxin Zhijian Co. Ltd.");
    VENDOR_TEXT(ecvendor_meysar_makina,                      "MEYSAR MAKINA ELEKTRONIK ENERJI YAZILIM SAN. TIC. LTD. STI");
    VENDOR_TEXT(ecvendor_all_lasersystem,                    "A.L.L. Lasersysteme GmbH");
    VENDOR_TEXT(ecvendor_suzhou_gfd,                         "Suzhou GFD Auto and Tech Co., Ltd");
    VENDOR_TEXT(ecvendor_l3_communications,                  "L-3 Communications, Communication Systems - West");
    VENDOR_TEXT(ecvendor_bnt,                                "BNT");
    VENDOR_TEXT(ecvendor_japan_radio,                        "Japan Radio Co., Ltd.");
    VENDOR_TEXT(ecvendor_schneider_motion_control,           "Schneider Electric SE");
    VENDOR_TEXT(ecvendor_parker_hannifin,                    "Parker Hannifin GmbH");
    VENDOR_TEXT(ecvendor_parker_hannifin_corp,               "Parker Hannifin Corporation");
    VENDOR_TEXT(ecvendor_advanced_systems_dev,               "Advanced Systems Development BVBA");
    VENDOR_TEXT(ecvendor_dsp,                                "DSP Automation");
    VENDOR_TEXT(ecvendor_compac_sorting,                     "Compac Sorting Equipment Ltd.");
    VENDOR_TEXT(ecvendor_gsk,                                "GSK CNC EQUIPMENT CO., LTD.");
    VENDOR_TEXT(ecvendor_fastech,                            "FASTECH Co., Ltd.");
    VENDOR_TEXT(ecvendor_sonotronic,                         "SONOTRONIC Nagel GmbH");
    VENDOR_TEXT(ecvendor_xian_mosvo,                         "XIAN MOSVO ELECTRONICS TECHNOLOGY CO.,LTD");
    VENDOR_TEXT(ecvendor_beckhoff_hardware,                  "Beckhoff Automation GmbH");
    VENDOR_TEXT(ecvendor_ed,                                 "ED Co., Ltd");
    VENDOR_TEXT(ecvendor_shanghai_sany_id1,                  "SHANGHAI SANY SCIENCE and TECHNOLOGY CO., LTD");
    VENDOR_TEXT(ecvendor_shanghai_sany_id2,                  "SHANGHAI SANY SCIENCE and TECHNOLOGY CO., LTD");
    VENDOR_TEXT(ecvendor_ga_drilling,                        "GA Drilling, Ltd.");
    VENDOR_TEXT(ecvendor_mettem,                             "METTEM-Specautomatic Ltd.");
    VENDOR_TEXT(ecvendor_tsinghua_uni_ee,                    "Tsinghua University, Department of Electronic Engineering");
    VENDOR_TEXT(ecvendor_control_z,                          "Control Z Corporation");
    VENDOR_TEXT(ecvendor_shangha_damon,                      "Shanghai Damon Logistics Technology Co.,LTD");
    VENDOR_TEXT(ecvendor_iba,                                "iba AG");
    VENDOR_TEXT(ecvendor_hengstler,                          "Hengstler GmbH");
    VENDOR_TEXT(ecvendor_lenord_bauer,                       "Lenord, Bauer and Co. GmbH");
    VENDOR_TEXT(ecvendor_university_bremen,                  "Universitaet Bremen, Institut fuer Automatisierungstechnik (IAT)");
    VENDOR_TEXT(ecvendor_shanghai_cohere,                    "Shanghai Cohere Electronics Technology Co., Ltd.");
    VENDOR_TEXT(ecvendor_robert_bosch,                       "Robert Bosch GmbH");
    VENDOR_TEXT(ecvendor_apdisar,                            "APDISAR (Association pour la Promotion et le Developpement de lAEcole DAIngenieurs ESISAR)");
    VENDOR_TEXT(ecvendor_convex,                             "Convex Co., Ltd.");
    VENDOR_TEXT(ecvendor_x2e,                                "X2E GmbH");
    VENDOR_TEXT(ecvendor_shanghai_sany_s,                    "SHANGHAI SANY SCIENCE and TECHNOLOGY CO., LTD");
    VENDOR_TEXT(ecvendor_sany_intelligent,                   "Sany Intelligent Control Equipment");
    VENDOR_TEXT(ecvendor_tu_darmstadt,                       "Technische Universitaet Darmstadt, Institut fuer Elektromechanische Konstruktionen");
    VENDOR_TEXT(ecvendor_mkt_systemtechnik,                  "MKT Systemtechnik GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_the_itaya,                          "The ITAYA Engineering Ltd.");
    VENDOR_TEXT(ecvendor_korea_elec_institute,               "Korea Electronics Technology Institute");
    VENDOR_TEXT(ecvendor_shanghai_sany_id3,                  "SHANGHAI SANY SCIENCE and TECHNOLOGY CO., LTD");
    VENDOR_TEXT(ecvendor_red_one,                            "Red one technologies");
    VENDOR_TEXT(ecvendor_applied_materials,                  "Applied Materials Inc.");
    VENDOR_TEXT(ecvendor_arca,                               "ARCA TECNOLOGIE srl");
    VENDOR_TEXT(ecvendor_aval,                               "AVAL DATA CORPORATION");
    VENDOR_TEXT(ecvendor_trust,                              "Trust Automation Inc.");
    VENDOR_TEXT(ecvendor_central_south_university,           "Central South University of Forestry and Technology, College of Computer Science and Information Technology");
    VENDOR_TEXT(ecvendor_dek_printing,                       "DEK Printing Machines Ltd.");
    VENDOR_TEXT(ecvendor_krebsforschungszentrum,             "Deutsches Krebsforschungszentrum");
    VENDOR_TEXT(ecvendor_elovis,                             "ELOVIS GmbH");
    VENDOR_TEXT(ecvendor_ektech,                             "EKTECH Elektronik");
    VENDOR_TEXT(ecvendor_shf_communication,                  "SHF Communication Technologies AG");
    VENDOR_TEXT(ecvendor_fms,                                "FMS (Flexible Manufacturing System)");
    VENDOR_TEXT(ecvendor_marcus_gossner,                     "Marcus Gossner SYSTEM SOLUTIONS");
    VENDOR_TEXT(ecvendor_grossenbacher,                      "Grossenbacher Systeme AG");
    VENDOR_TEXT(ecvendor_hstar,                              "Hstar Technologies Corp.");
    VENDOR_TEXT(ecvendor_university_western_swiss,           "University of Applied Sciences Western Switzerland, Institute of Systems Engineering");
    VENDOR_TEXT(ecvendor_nti_linmot,                         "NTI AG - LinMot");
    VENDOR_TEXT(ecvendor_lam_research,                       "Lam Research Corporation");
    VENDOR_TEXT(ecvendor_mlt_micro,                          "MLT Micro Laser Technology GmbH");
    VENDOR_TEXT(ecvendor_tu_braunschweig,                    "Technische Universitaet Braunschweig");
    VENDOR_TEXT(ecvendor_power_automation,                   "Power Automation GmbH");
    VENDOR_TEXT(ecvendor_rafi,                               "RAFI GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_steinbeis,                          "Steinbeis-Transferzentrum Systemtechnik");
    VENDOR_TEXT(ecvendor_tecan,                              "Tecan Schweiz AG");
    VENDOR_TEXT(ecvendor_tews,                               "TEWS Elektronik GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_timax,                              "Timax Electronics and Machinery Ltd.");
    VENDOR_TEXT(ecvendor_olympus,                            "OLYMPUS CORPORATION");
    VENDOR_TEXT(ecvendor_siasun_robot,                       "SIASUN Robot and Automation Co., Ltd.");
    VENDOR_TEXT(ecvendor_green_field,                        "Green Field Control System (I) Pvt. Ltd.");
    VENDOR_TEXT(ecvendor_jt3,                                "JT3, LLC");
    VENDOR_TEXT(ecvendor_volvo,                              "Volvo Group");
    VENDOR_TEXT(ecvendor_megatec,                            "megatec electronic GmbH");
    VENDOR_TEXT(ecvendor_arte_motion,                        "Arte Motion S.p.A.");
    VENDOR_TEXT(ecvendor_chinese_academy_ioe,                "Chinese Academy of Sciences, Institute of Optics and Electronics (IOE)");
    VENDOR_TEXT(ecvendor_shenzhen_just_motion,               "Shenzhen Just Motion Control Electromechanics Co.,Ltd");
    VENDOR_TEXT(ecvendor_paul_maschinenfabrik,               "PAUL Maschinenfabrik GmbH and Co.KG");
    VENDOR_TEXT(ecvendor_mesacon,                            "Mesacon Messelektronik GmbH Dresden");
    VENDOR_TEXT(ecvendor_shanghai_tech_full,                 "Shanghai Tech Full Electric Co., Ltd.");
    VENDOR_TEXT(ecvendor_shandong_university,                "Shandong University, School of Electrical Engineering");
    VENDOR_TEXT(ecvendor_scandinova,                         "ScandiNova Systems AB");
    VENDOR_TEXT(ecvendor_woojin_plaimm,                      "Woojin Plaimm Co., Ltd");
    VENDOR_TEXT(ecvendor_robocubetech,                       "ROBOCUBETECH Co., Ltd");
    VENDOR_TEXT(ecvendor_shanghai_step,                      "Shanghai STEP Electric Corporation");
    VENDOR_TEXT(ecvendor_sunin,                              "Sunin Technology Inc.");
    VENDOR_TEXT(ecvendor_comizoa,                            "COMIZOA Co., Ltd.");
    VENDOR_TEXT(ecvendor_ruchservomotor,                     "Ruchservomotor Ltd.");
    VENDOR_TEXT(ecvendor_dalian_jafeng,                      "Dalian Jafeng Electronics Co., Ltd.");
    VENDOR_TEXT(ecvendor_addi_data,                          "ADDI-DATA GmbH");
    VENDOR_TEXT(ecvendor_husky_injection,                    "Husky Injection Molding Systems Ltd.");
    VENDOR_TEXT(ecvendor_jinoid,                             "JINOID CO., LTD.");
    VENDOR_TEXT(ecvendor_bertec,                             "Bertec Corporation");
    VENDOR_TEXT(ecvendor_innovasic,                          "Innovasic Inc.");
    VENDOR_TEXT(ecvendor_puloonc,                            "PULOON Technology Inc.");
    VENDOR_TEXT(ecvendor_handtmann,                          "Albert Handtmann Maschinenfabrik GmbH and Co. KG");
    VENDOR_TEXT(ecvendor_dewesoft,                           "Dewesoft d.o.o.");

#endif /* !EC_DEMO_TINY */
    default:
        if (0xE0000000 == (((EC_T_DWORD)eVendorId)&0xE0000000))
        {
            pRet = SlaveVendorText((T_eEtherCAT_Vendor)(((EC_T_DWORD)eVendorId)&(~0xE0000000)));
        }
        else
        {
            pRet = "Unknown";
        }
    }

    return pRet;
}

/***************************************************************************************************/
/**
 * \brief  Slave Product Code Text.
 *
 * \return PC Text.
 */
const EC_T_CHAR* SlaveProdCodeText(T_eEtherCAT_Vendor eVendorId, T_eEtherCAT_ProductCode eProductCode)
{
    const EC_T_CHAR* pRet = EC_NULL;

    switch (eVendorId)
    {
    case ecvendor_beckhoff:
        {
            switch (eProductCode)
            {
            case ecprodcode_beck_AX2000_B110:   pRet  = "AX2000"; break;
         /* case ecprodcode_beck_AX2000_B110:   pRet  = "AX2000 B110"; break; */
         /* case ecprodcode_beck_AX2000_B120:   pRet  = "AX2000 B120"; break; */
            case ecprodcode_beck_BK1120:        pRet  = "BK1120"; break;
            case ecprodcode_beck_CX1100_0004:   pRet  = "CX1100 0004"; break;
            case ecprodcode_beck_CU1128:        pRet  = "CU1128"; break;
            case ecprodcode_beck_EK1100:        pRet  = "EK1100"; break;
            case ecprodcode_beck_EK1101:        pRet  = "EK1101"; break;
            case ecprodcode_beck_EK1110:        pRet  = "EK1110"; break;
            case ecprodcode_beck_EK1122:        pRet  = "EK1122"; break;
            case ecprodcode_beck_EK1814:        pRet  = "EK1814"; break;
            case ecprodcode_beck_EK1818:        pRet  = "EK1818"; break;
            case ecprodcode_beck_EK1828:        pRet  = "EK1828"; break;
            case ecprodcode_beck_EK1914:        pRet  = "EK1914"; break;
            case ecprodcode_beck_EL1002:        pRet  = "EL1002"; break;
            case ecprodcode_beck_EL1004:        pRet  = "EL1004"; break;
         /* case ecprodcode_beck_EL1004_0010:   pRet  = "EL1004 0010"; break; */
            case ecprodcode_beck_EL1008:        pRet  = "EL1008"; break;
            case ecprodcode_beck_EL1012:        pRet  = "EL1012"; break;
            case ecprodcode_beck_EL1014:        pRet  = "EL1014"; break;
         /* case ecprodcode_beck_EL1014_0010:   pRet  = "EL1014 0010"; break; */
            case ecprodcode_beck_EL1018:        pRet  = "EL1018"; break;
            case ecprodcode_beck_EL1034:        pRet  = "EL1034"; break;
            case ecprodcode_beck_EL1094:        pRet  = "EL1094"; break;
            case ecprodcode_beck_EL1252:        pRet  = "EL1252"; break;
            case ecprodcode_beck_EL1259:        pRet  = "EL1259"; break;
            case ecprodcode_beck_EL1262:        pRet  = "EL1262"; break;
            case ecprodcode_beck_EL1904:        pRet  = "EL1904"; break;
            case ecprodcode_beck_EL1889:        pRet  = "EL1889"; break;

            case ecprodcode_beck_EL2002:        pRet  = "EL2002"; break;
            case ecprodcode_beck_EL2004:        pRet  = "EL2004"; break;
            case ecprodcode_beck_EL2008:        pRet  = "EL2008"; break;
            case ecprodcode_beck_EL2032:        pRet  = "EL2032"; break;
            case ecprodcode_beck_EL2262:        pRet  = "EL2262"; break;
            case ecprodcode_beck_EL2252:        pRet  = "EL2252"; break;
            case ecprodcode_beck_EL2502:        pRet  = "EL2502"; break;
            case ecprodcode_beck_EL2521:        pRet  = "EL2521"; break;
         /* case ecprodcode_beck_EL2521_1001:   pRet  = "EL2521 0001"; break; */
            case ecprodcode_beck_EL2889:        pRet  = "EL2889"; break;
            case ecprodcode_beck_EL2904:        pRet  = "EL2904"; break;
            case ecprodcode_beck_EL3102:        pRet  = "EL3102"; break;
            case ecprodcode_beck_EL3112:        pRet  = "EL3112"; break;
            case ecprodcode_beck_EL3122:        pRet  = "EL3122"; break;
            case ecprodcode_beck_EL3142:        pRet  = "EL3142"; break;
         /* case ecprodcode_beck_EL3142_0010:   pRet  = "EL3142 0010"; break; */
            case ecprodcode_beck_EL3152:        pRet  = "EL3152"; break;
            case ecprodcode_beck_EL3162:        pRet  = "EL3162"; break;
            case ecprodcode_beck_EL3202:        pRet  = "EL3202"; break;
            case ecprodcode_beck_EL3312:        pRet  = "EL3312"; break;
            case ecprodcode_beck_EL3356:        pRet  = "EL3356"; break;
            case ecprodcode_beck_EL3702:        pRet  = "EL3702"; break;
            case ecprodcode_beck_EL4002:        pRet  = "EL4002"; break;
            case ecprodcode_beck_EL4102:        pRet  = "EL4102"; break;
            case ecprodcode_beck_EL4112:        pRet  = "EL4112"; break;
         /* case ecprodcode_beck_EL4112_0010:   pRet  = "EL4112 0010"; break; */
            case ecprodcode_beck_EL4122:        pRet  = "EL4122"; break;
            case ecprodcode_beck_EL4132:        pRet  = "EL4132"; break;
            case ecprodcode_beck_EL5001:        pRet  = "EL5001"; break;
            case ecprodcode_beck_EL5101:        pRet  = "EL5101"; break;
            case ecprodcode_beck_EL5151:        pRet  = "EL5151"; break;
            case ecprodcode_beck_EL5152:        pRet  = "EL5152"; break;
            case ecprodcode_beck_EL6001:        pRet  = "EL6001"; break;
            case ecprodcode_beck_EL6021:        pRet  = "EL6021"; break;
            case ecprodcode_beck_EL6601:        pRet  = "EL6601"; break;
            case ecprodcode_beck_EL6614:        pRet  = "EL6614"; break;
            case ecprodcode_beck_EL6690:        pRet  = "EL6690"; break;
            case ecprodcode_beck_EL6731:        pRet  = "EL6731"; break;
         /* case ecprodcode_beck_EL6731_0010:   pRet  = "EL6731 0010"; break; */
            case ecprodcode_beck_EL6751:        pRet  = "EL6751"; break;
            case ecprodcode_beck_EL6752:        pRet  = "EL6752"; break;

            case ecprodcode_beck_EL6900:        pRet  = "EL6900"; break;
            case ecprodcode_beck_EL6910:        pRet  = "EL6910"; break;
            case ecprodcode_beck_EL6930:        pRet  = "EL6930"; break;
            case ecprodcode_beck_EL9505:        pRet  = "EL9505"; break;
            case ecprodcode_beck_EL9510:        pRet  = "EL9510"; break;
            case ecprodcode_beck_EL9512:        pRet  = "EL9512"; break;
            case ecprodcode_beck_EL9800:        pRet  = "EL9800"; break;
            case ecprodcode_beck_EL9820:        pRet  = "EL9820"; break;
            case ecprodcode_beck_FM5001:        pRet  = "FM5001"; break;

            default:                            pRet  = "Unknown"; break;
            }
        } break;
#if (!defined EC_DEMO_TINY)
    case ecvendor_scuola_superiore_s_anna:
        {
            pRet  = "Unknown";
        } break;
    case ecvendor_ixxat:
        {
            switch (eProductCode)
            {
            case ecprodcode_ixx_iem:            pRet  = "IEM"; break;
            case ecprodcode_ixx_ETCio100:       pRet  = "ETCio 100"; break;
            default:                            pRet  = "Unknown"; break;
            }
        } break;
    case ecvendor_pollmeier:
        {
            switch (eProductCode)
            {
            case ecprodcode_esr_Trio_1:
            case ecprodcode_esr_Trio:           pRet  = "TrioDrive"; break;
            case ecprodcode_esr_Midi:           pRet  = "MidiDrive"; break;
            default:                            pRet  = "Unknown"; break;
            }
        } break;
    case ecvendor_kuebler:
        {
            switch (eProductCode)
            {
            case ecprodcode_kuebler_Multiturn5868:
                pRet  = "Multiturn 5868"; break;
            default:                            pRet  = "Unknown"; break;
            }
        } break;
    case ecvendor_keb:
        {
            switch (eProductCode)
            {
            case ecprodcode_keb_KEB1736F5_3d:   pRet  = "KEB1736F5 3.d"; break;
            case ecprodcode_keb_EcatGateway:    pRet  = "KEB ECAT Gateway"; break;
            default:                            pRet  = "Unknown"; break;
            };
        } break;
    case ecvendor_moog:
        {
            switch (eProductCode)
            {
            case ecprodcode_moog_anin:          pRet  = "Analogue In"; break;
            case ecprodcode_moog_ServoValveD671:pRet  = "Servo Valve D671"; break;
            default:                            pRet  = "Unknown"; break;
            };
        } break;
    case ecvendor_lenze:
        {
            switch (eProductCode)
            {
            case ecprodcode_ldc_el9400:         pRet  = "EL9400 L-Force Servo Drive"; break;
            case ecprodcode_ldc_e94aycet:       pRet  = "E94AYCET L-Force Servo Drive"; break;
            case ecprodcode_ldc_servogun2:      pRet  = "2 Axis Servo Gun"; break;
            case ecprodcode_ldc_servogun3:      pRet  = "3 Axis Servo Gun"; break;
            case ecprodcode_ldc_epms130:        pRet  = "EPM-S130"; break;
            case ecprodcode_ldc_el8400:         pRet  = "EL8400 L-Force Servo Drive"; break;
            case ecprodcode_ldc_stateline:      pRet  = "Stateline L-Force Servo Drive"; break;
            default:                            pRet  = "Unknown"; break;
            };
        } break;
    case ecvendor_hilscher:
        {
            switch (eProductCode)
            {
            case ecprodcode_hil_NXSB100:        pRet  = "NX SB100"; break;
            case ecprodcode_hil_NXSB100DC:      pRet  = "NX SB100 DC"; break;
            default:                            pRet  = "Unknown"; break;
            };
        } break;
    case ecvendor_renesas:
        {
            switch (eProductCode)
            {
            case ecprodcode_ren_EC1:            pRet = "EC-1"; break;
            default:                            pRet = "Unknown"; break;
            };
        } break;
    case ecvendor_komax:
        {
            switch (eProductCode)
            {
            case ecprodcode_sh_ACSE:            pRet  = "ACSE"; break;
            default:                            pRet  = "Unknown"; break;
            };;
        } break;
    case ecvendor_sew_eurodrive:
        {
            switch (eProductCode)
            {
            case ecprodcode_sew_movidrive:      pRet  = "Movi Drive"; break;
            case ecprodcode_sew_moviaxis:       pRet  = "Movi Axis"; break;
            default:                            pRet  = "Unknown"; break;
            };
        } break;
    case ecvendor_danaher:
        {
            switch (eProductCode)
            {
            case ecprodcode_dan_servostar300:   pRet  = "ServoStar 300"; break;
            default:                            pRet  = "Unknown"; break;
            };
        } break;
    case ecvendor_controltechniques:
        {
            switch (eProductCode)
            {
            case ecprodcode_ct_drive:           pRet  = "Emerson Drive"; break;
            case ecprodcode_ct_drive_sp:        pRet  = "Emerson Drive SP"; break;
            default:                            pRet  = "Unknown"; break;
            };
        } break;
    case ecvendor_smc:
        {
            switch (eProductCode)
            {
            case ecprodcode_smc_serifcunit:     pRet  = "Serif C Unit"; break;
            default:                            pRet  = "Unknown"; break;
            };
        } break;
    case ecvendor_jumo:
        {
            switch (eProductCode)
            {
            case ecprodcode_jumo_Basis:         pRet  = "Basismodul"; break;
            case ecprodcode_jumo_Busskoppler:   pRet  = "Busskoppler"; break;
            case ecprodcode_jumo_HMI:           pRet  = "HMI"; break;
            case ecprodcode_jumo_Router:        pRet  = "Router"; break;
            case ecprodcode_jumo_BIO:           pRet  = "Basic I/O"; break;
            case ecprodcode_jumo_RELAIS:        pRet  = "Relais"; break;
            case ecprodcode_jumo_REGLER:        pRet  = "Regler"; break;
            case ecprodcode_jumo_Analog_4_Ein:  pRet  = "4 x Analog Eingang"; break;
            case ecprodcode_jumo_Analog_8_Ein:  pRet  = "8 x Analog Eingang"; break;
            case ecprodcode_jumo_Analog_4_Aus:  pRet  = "4 x Analog Ausgang"; break;
            case ecprodcode_jumo_Analog_8_Aus:  pRet  = "8 x Analog Ausgang"; break;
            default:                            pRet  = "Unknown"; break;
            };
        } break;
    case ecvendor_baumueller:
        {
            switch (eProductCode)
            {
            case ecprodcode_baumueller_BM3000:  pRet  = "BM3000"; break;
            case ecprodcode_baumueller_BM4000:  pRet  = "BM4000"; break;
            default:                            pRet  = "Unknown"; break;
            };
        } break;
    case ecvendor_deutschmann:
        {
            switch (eProductCode)
            {
            case ecprodcode_dm_rs232gw:         pRet  = "RS232 Gateway"; break;
            default:                            pRet  = "Unknown"; break;
            };
        } break;
    case ecvendor_parker:
        {
            switch (eProductCode)
            {
            case ecprodcode_par_drive:          pRet  = "Drive"; break;
            default:                            pRet  = "Unknown"; break;
            };
        } break;
    case ecvendor_national_instruments:
        {
            switch (eProductCode)
            {
            case ecprodcode_ni_digio:           pRet  = "Digital I/O"; break;
            case ecprodcode_ni_anaio:           pRet  = "Analog I/O"; break;
            default:                            pRet  = "Unknown"; break;
            };
        } break;
    case ecvendor_idam:
        {
            switch (eProductCode)
            {
            case ecprodcode_idam_DSMRW:         pRet  = "DSMRW"; break;
            default:                            pRet  = "Unknown"; break;
            }
        } break;
    case ecvendor_baumer_th:
        {
            pRet  = "Unknown";
        } break;
    case ecvendor_tr:
        {
            switch (eProductCode)
            {
            case ecprodcode_tr_linencoder2:     pRet  = "Line Encoder 2"; break;
            default:                            pRet  = "Unknown"; break;
            }
        } break;
    case ecvendor_bce:
        {
            switch (eProductCode)
            {
            case ecprodcode_bce_AMAT_Handbox:   pRet  = "AMAT Handbox"; break;
            case ecprodcode_bce_AMAT_HB_digin:  pRet  = "AMAT Handbox Digital In"; break;
            case ecprodcode_bce_AMAT_HB_digout: pRet  = "AMAT Handbox Digital Out"; break;
            default:                            pRet  = "Unknown"; break;
            }
        } break;
    case ecvendor_koenig:
        {
            switch (eProductCode)
            {
            case ecprodcode_kng_pc104:          pRet  = "PC104"; break;
            default:                            pRet  = "Unknown"; break;
            }
        } break;
    case ecvendor_acontis:
        {
            switch (eProductCode)
            {
            case ecprodcode_at_atem:            pRet  = "EtherCAT Master"; break;
            case ecprodcode_at_atemTestSlave:   pRet  = "EtherCAT TestSlave"; break;
            default:                            pRet  = "Unknown"; break;
            }
        } break;
    case ecvendor_kuka:
        {
            switch (eProductCode)
            {
            case ecprodcode_kr_cib:             pRet  = "CIB"; break;
            case ecprodcode_kr_rdc3:            pRet  = "RDC 3 angle resolver"; break;
            case ecprodcode_kr_cibsion:         pRet  = "CIB-SION"; break;
            case ecprodcode_kr_sionkpp:         pRet  = "SION-KPP"; break;
            case ecprodcode_kr_sionksp:         pRet  = "SION-KSP"; break;
            default:                            pRet  = "Unknown"; break;
            }
        } break;
    case ecvendor_kuhnke:
        {
            switch (eProductCode)
            {
            case ecprodcode_kuh_VFIO_BK:        pRet  = "BK"; break;
            case ecprodcode_kuh_VFIO_DIO:       pRet  = "Digital I/O"; break;
            default:                            pRet  = "Unknown"; break;
            }
        } break;
    case ecvendor_jat:
        {
            switch (eProductCode)
            {
            case ecprodcode_jat_drive1:         pRet  = "Drive"; break;
            default:                            pRet  = "Unknown"; break;
            }
        } break;
    case ecvendor_festo:
        {
            switch (eProductCode)
            {
            case ecprodcode_fst_cpx:            pRet  = "CPX"; break;
            default:                            pRet  = "Unknown"; break;
            }
        } break;
    case ecvendor_copley:
        {
            switch (eProductCode)
            {
            case ecprodcode_cpl_accelnet:       pRet  = "Accelnet Servo Drive"; break;
            default:                            pRet  = "Unknown"; break;
            }
        } break;
    case ecvendor_robox:
        {
            switch (eProductCode)
            {
            case ecprodcode_rx_coedrivegw:      pRet  = "CoE Drive Gateway"; break;
            default:                            pRet  = "Unknown"; break;
            }
        } break;
    case ecvendor_dresdenelektronik:
        {
            switch (eProductCode)
            {
            case ecprodcode_de_sdac3100:        pRet  = "de axis controller 3100"; break;
            default:                            pRet  = "Unknown"; break;
            };
        } break;
    case ecvendor_yaskawa:
        {
            switch (eProductCode)
            {
            case ecprodcode_yas_sgdv_e1:        pRet  = "SGDV-E1 CoE Drive"; break;
            default:                            pRet  = "Unknown"; break;
            };
        } break;
    case ecvendor_omron:
        {
            switch (eProductCode)
            {
            case ecprodcode_omron_GX_ID1621:     pRet = "GX-ID1621"; break;
            case ecprodcode_omron_GX_OD1621:     pRet = "GX-OD1621"; break;
            case ecprodcode_omron_GX_JC06_Main:  pRet = "GX-JC06(IN,X2,X3) Main"; break;
            case ecprodcode_omron_GX_JC06_Sub:   pRet = "GX-JC06(X4,X5,X6) Sub"; break;
            case ecprodcode_omron_GX_JC06H_Main: pRet = "GX-JC06-H(IN,X2,X3) Main"; break;
            case ecprodcode_omron_GX_JC06H_Sub:  pRet = "GX-JC06-H(X4,X5,X6) Sub"; break;
            default:                             pRet = "Unknown"; break;
            };
        } break;
#endif /* !EC_DEMO_TINY */
    default:
        if (0xE0000000 == (((EC_T_DWORD)eVendorId)&0xE0000000))
        {
            pRet = SlaveProdCodeText((T_eEtherCAT_Vendor)(((EC_T_DWORD)eVendorId)&(~0xe0000000)), eProductCode);
        }
        else
        {
            pRet = "Unknown";
        }
    }

    return pRet;
}

/***************************************************************************************************/
/**
 * \brief  Create ESC Type string from Type, Slave Register Offset 0x00.
 *
 * \return Name of ESC type.
 */
const EC_T_CHAR* ESCTypeText(EC_T_BYTE   byESCType)
{
    const EC_T_CHAR* pszRet = EC_NULL;

    switch (byESCType)
    {
    case ESCTYPE_BKHF_ELOLD:            pszRet  = "Beckhoff ELs (Old)"; break;
    case ESCTYPE_ESC10|ESCTYPE_ESC20:   pszRet  = "Beckhoff FPGA ESC 10/20"; break;
    case ESCTYPE_IPCORE:                pszRet  = "IPCORE"; break;
    case ESCTYPE_ET1100:                pszRet  = "Beckhoff ET1100"; break;
    case ESCTYPE_ET1200:                pszRet  = "Beckhoff ET1200"; break;
    case ESCTYPE_NETX100_500:           pszRet  = "Hilscher NetX 100/500"; break;
    case ESCTYPE_NETX50:                pszRet  = "Hilscher NetX 50"; break;
    case ESCTYPE_NETX5:                 pszRet  = "Hilscher NetX 5"; break;
    case ESCTYPE_NETX51_52:             pszRet  = "Hilscher NetX 51/52"; break;
    case ESCTYPE_NETX90:                pszRet  = "Hilscher NetX 90"; break;
    case ESCTYPE_TI_SITARA:             pszRet  = "Texas Instruments Sitara"; break;
    case ESCTYPE_TI_C2000:              pszRet  = "Texas Instruments C2000"; break;
    case ESCTYPE_INFINEON:              pszRet  = "Infineon"; break;
    case ESCTYPE_RENESAS:               pszRet  = "Renesas"; break;
    case ESCTYPE_INNOVASIC:             pszRet  = "Innovasic"; break;
    case ESCTYPE_HMS:                   pszRet  = "HMS"; break;
    case ESCTYPE_MICROCHIP:             pszRet  = "Microchip"; break;
    case ESCTYPE_TRINAMIC:              pszRet  = "Trinamic"; break;
    default:                            pszRet  = "Unknown Slave Controller"; break;
    }

    return pszRet;
}
/*-END OF SOURCE FILE--------------------------------------------------------*/
