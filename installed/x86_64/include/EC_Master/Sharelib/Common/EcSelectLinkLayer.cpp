/*-----------------------------------------------------------------------------
 * EcSelectLinkLayer.cpp
 * Copyright                acontis technologies GmbH, Ravensburg, Germany
 * Response                 Paul Bussmann
 * Description              EC-Master link layer selection
 *---------------------------------------------------------------------------*/

/*-LOGGING-------------------------------------------------------------------*/
#ifndef pEcLogParms
extern struct _EC_T_LOG_PARMS G_aLogParms[];
#define pEcLogParms (&G_aLogParms[0])
#endif

/*-INCLUDES------------------------------------------------------------------*/
#include "EcDemoApp.h"

/*-DEFINES-------------------------------------------------------------------*/

/*-FUNCTION-DEFINITIONS------------------------------------------------------*/
/********************************************************************************/
/** \brief  Parse IP address from command line to byte array (EC_T_BYTE[4])
 *
 * \return  EC_TRUE in case of success, EC_FALSE in case of an invalid parameter
 */
EC_T_BOOL ParseIpAddress
(
    EC_T_CHAR* ptcWord,
    EC_T_BYTE* pbyIpAddress)
{
    EC_T_CHAR* ptcTmp   = EC_NULL;
    EC_T_INT   nCnt     = 0;
    EC_T_BOOL bRetVal = EC_TRUE;

    if (EC_NULL == pbyIpAddress)
    {
        bRetVal = EC_FALSE;
        goto Exit;
    }

    /* Get IP address */
    ptcTmp = ptcWord;
    for (nCnt = 0; nCnt < 4; nCnt++)
    {
        if (ptcTmp == EC_NULL)
        {
            bRetVal = EC_FALSE;
            goto Exit;
        }
        pbyIpAddress[nCnt] = (EC_T_BYTE)OsStrtol(ptcTmp, EC_NULL, 10);
        if (nCnt < 2)
        {
            ptcTmp = OsStrtok(EC_NULL, ".");
        }
        else if (nCnt < 3)
        {
            ptcTmp = OsStrtok(EC_NULL, " ");
        }
    }

Exit:
    return bRetVal;
}

/********************************************************************************/
/** \brief  Parse MAC address from string to byte array (EC_T_BYTE[6])
 *
 * \return  EC_TRUE in case of success, EC_FALSE in case of an invalid parameter
 */
EC_T_BOOL ParseMacAddress
(
    EC_T_CHAR* szMac,
    EC_T_BYTE* pbyMac)
{
    EC_T_CHAR  abyMacEntry[3];
    EC_T_INT   nCnt = 0;
    EC_T_BOOL  bRetVal = EC_TRUE;

    OsMemset(abyMacEntry, 0, 3);

    /* Get MAC address */
    if (OsStrlen(szMac) != OsStrlen("xx-xx-xx-xx-xx-xx"))
    {
        bRetVal = EC_FALSE;
        goto Exit;
    }

    for (nCnt = 0; nCnt < 6; nCnt++)
    {
        if ((nCnt < 5) && ((szMac[nCnt * 3 + 2] != '-') && (szMac[nCnt * 3 + 2] != ':')))
        {
            bRetVal = EC_FALSE;
            goto Exit;
        }

        abyMacEntry[0] = szMac[nCnt * 3];
        abyMacEntry[1] = szMac[nCnt * 3 + 1];
        pbyMac[nCnt] = (EC_T_BYTE)OsStrtol(abyMacEntry, EC_NULL, 16);
    }

Exit:
    return bRetVal;
}

/********************************************************************************/
/** Parse next command line argument
 *
 * Return: pointer to next argument
*/
EC_T_CHAR* GetNextWord(EC_T_CHAR **ppCmdLine, EC_T_CHAR *pStorage)
{
    EC_T_CHAR *pWord;

    EC_UNREFPARM(ppCmdLine);
    EC_UNREFPARM(pStorage);

    pWord = (EC_T_CHAR *)OsStrtok(NULL, " ");

    return pWord;
}

/***************************************************************************************************/
/**
\brief  Parses string parameter value from command line

\return EC_TRUE if successfully parsed, EC_FALSE on syntax errors
*/
static EC_T_BOOL ParseString(
    EC_T_CHAR**     ptcWord,
    EC_T_CHAR**     lpCmdLine,
    EC_T_CHAR*      tcStorage)
{
    *ptcWord = GetNextWord(lpCmdLine, tcStorage);
    if ((*ptcWord == EC_NULL) || (OsStrncmp(*ptcWord, "-", 1) == 0))
    {
        return EC_FALSE;
    }

    return EC_TRUE;
}

#if (defined INCLUDE_EMLLPROXY)
/***************************************************************************************************/
/**
\brief  Parses EC_T_WORD parameter value from command line

\return EC_TRUE if successfully parsed, EC_FALSE on syntax errors
*/
static EC_T_BOOL ParseWord(
    EC_T_CHAR**     ptcWord,
    EC_T_CHAR**     lpCmdLine,
    EC_T_CHAR*      tcStorage,
    EC_T_WORD*      pwValue)
{
    if (!ParseString(ptcWord, lpCmdLine, tcStorage))
    {
        return EC_FALSE;
    }
    *pwValue = (EC_T_WORD)OsStrtol(*ptcWord, NULL, 0);

    return EC_TRUE;
}
#endif

/***************************************************************************************************/
/**
\brief  Parses EC_T_DWORD parameter value from command line

\return EC_TRUE if successfully parsed, EC_FALSE on syntax errors
*/
static EC_T_BOOL ParseDword(
    EC_T_CHAR**     ptcWord,
    EC_T_CHAR**     lpCmdLine,
    EC_T_CHAR*      tcStorage,
    EC_T_DWORD*     pdwValue)
{
    if (!ParseString(ptcWord, lpCmdLine, tcStorage))
    {
        return EC_FALSE;
    }
    *pdwValue = (EC_T_DWORD)OsStrtol(*ptcWord, NULL, 0);

    return EC_TRUE;
}

/***************************************************************************************************/
/**
\brief  Parses EC_T_LINKMODE parameter value from command line

\return EC_TRUE if successfully parsed, EC_FALSE on syntax errors
*/
EC_T_BOOL ParseLinkMode(
    EC_T_CHAR**     ptcWord,
    EC_T_CHAR**     lpCmdLine,
    EC_T_CHAR*      tcStorage,
    EC_T_LINKMODE*  peLinkMode)
{
    EC_T_DWORD dwMode = 0;
    EC_T_BOOL bRes = EC_FALSE;

    if (ParseDword(ptcWord, lpCmdLine, tcStorage, &dwMode))
    {
        if (dwMode == 0)
        {
            *peLinkMode = EcLinkMode_INTERRUPT;
            bRes = EC_TRUE;
        }
        else if (dwMode == 1)
        {
            *peLinkMode = EcLinkMode_POLLING;
            bRes = EC_TRUE;
        }
        else
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Invalid Link Layer Mode (mode == %d)\n", dwMode));
        }
    }
    return bRes;
}

/***************************************************************************************************/
/**
\brief  Fill common link layer parameters
*/
static EC_T_VOID LinkParmsInit(EC_T_LINK_PARMS* pLinkParms,
                        const EC_T_DWORD dwSignature, const EC_T_DWORD dwSize, const char* szDriverIdent,
                        const EC_T_DWORD dwInstance, const EC_T_LINKMODE eLinkMode, const EC_T_DWORD dwIstPriority = 0)
{
    OsMemset(pLinkParms, 0, sizeof(EC_T_LINK_PARMS));
    pLinkParms->dwSignature = dwSignature;
    pLinkParms->dwSize = dwSize;
    OsStrncpy(pLinkParms->szDriverIdent, szDriverIdent, EC_DRIVER_IDENT_MAXLEN);
    pLinkParms->dwInstance = dwInstance;
    pLinkParms->eLinkMode = eLinkMode;
    pLinkParms->dwIstPriority = dwIstPriority;
}

#if (defined INCLUDE_EMLLANTAIOS)
/***************************************************************************************************/
/**
\brief  Create ANTAIOS link layer parameters according to current command line parsing

\return EC_E_NOERROR     if link layer parameters was created
EC_E_NOTFOUND    if command line was not matching
EC_E_INVALIDPARM if syntax error
*/
static EC_T_DWORD CreateLinkParmsFromCmdLineAntaios(EC_T_CHAR** ptcWord, EC_T_CHAR** lpCmdLine, EC_T_CHAR* tcStorage, EC_T_BOOL* pbGetNextWord,
    EC_T_LINK_PARMS** ppLinkParms)
{
    EC_T_DWORD dwRetVal = EC_E_ERROR;
    EC_T_LINK_PARMS_ANTAIOS* pLinkParmsAdapter = EC_NULL;

    EC_UNREFPARM(pbGetNextWord);

    /* check for matching adapter */
    if (OsStricmp(*ptcWord, "-antaios") != 0)
    {
        dwRetVal = EC_E_NOTFOUND;
        goto Exit;
    }
    /* alloc adapter specific link parms */
    pLinkParmsAdapter = (EC_T_LINK_PARMS_ANTAIOS*)OsMalloc(sizeof(EC_T_LINK_PARMS_ANTAIOS));
    if (EC_NULL == pLinkParmsAdapter)
    {
        dwRetVal = EC_E_NOMEMORY;
        goto Exit;
    }

    OsMemset(pLinkParmsAdapter, 0, sizeof(EC_T_LINK_PARMS_ANTAIOS));
    LinkParmsInit(&pLinkParmsAdapter->linkParms, EC_LINK_PARMS_SIGNATURE_ANTAIOS, sizeof(EC_T_LINK_PARMS_ANTAIOS), EC_LINK_PARMS_IDENT_ANTAIOS, 2, EcLinkMode_POLLING);

    /* no errors */
    *ppLinkParms = &pLinkParmsAdapter->linkParms;
    dwRetVal = EC_E_NOERROR;

Exit:
    if (EC_E_NOERROR != dwRetVal)
    {
        SafeOsFree(pLinkParmsAdapter);
    }
    return dwRetVal;
}
#endif /* INCLUDE_EMLLANTAIOS */

#if (defined INCLUDE_EMLLALTERATSE)

/***************************************************************************************************/
/**
\brief  Create Altera TSE link layer parameters according to current command line parsing

\return EC_E_NOERROR     if link layer parameters was created
EC_E_NOTFOUND    if command line was not matching
EC_E_INVALIDPARM if syntax error
*/
static EC_T_DWORD CreateLinkParmsFromCmdLineAlteraTse(EC_T_CHAR** ptcWord,
    EC_T_CHAR** lpCmdLine, EC_T_CHAR* tcStorage,
    EC_T_BOOL*  pbGetNextWord,
    EC_T_LINK_PARMS** ppLinkParms
)
{
    EC_T_DWORD dwRetVal = EC_E_ERROR;
    EC_T_LINK_PARMS_ALTERATSE* pLinkParmsAdapter = EC_NULL;

    EC_UNREFPARM(pbGetNextWord);

    /* check for matching adapter */
    if (OsStricmp(*ptcWord, "-alteratse") != 0)
    {
        dwRetVal = EC_E_NOTFOUND;
        goto Exit;
    }
    /* alloc adapter specific link parms */
    pLinkParmsAdapter = (EC_T_LINK_PARMS_ALTERATSE*)OsMalloc(sizeof(EC_T_LINK_PARMS_ALTERATSE));
    if (EC_NULL == pLinkParmsAdapter)
    {
        dwRetVal = EC_E_NOMEMORY;
        goto Exit;
    }
    OsMemset(pLinkParmsAdapter, 0, sizeof(EC_T_LINK_PARMS_ALTERATSE));
    LinkParmsInit(&pLinkParmsAdapter->linkParms, EC_LINK_PARMS_SIGNATURE_ALTERATSE,
        sizeof(EC_T_LINK_PARMS_ALTERATSE), EC_LINK_PARMS_IDENT_ALTERATSE, 1, EcLinkMode_POLLING);

    if (!ParseDword(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.dwInstance)
        || !ParseLinkMode(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.eLinkMode)
        )
    {
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }

    if (pLinkParmsAdapter->linkParms.dwInstance <= 1 || pLinkParmsAdapter->linkParms.dwInstance > 2)
    {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Port number must be between 1 and 2\n"));
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }

    pLinkParmsAdapter->abyMac[0] = 0x01;
    pLinkParmsAdapter->abyMac[1] = 0xB4;
    pLinkParmsAdapter->abyMac[2] = 0xC3;
    pLinkParmsAdapter->abyMac[3] = 0xDD;
    pLinkParmsAdapter->abyMac[4] = 0xEE;
    pLinkParmsAdapter->abyMac[5] = 0xFF;

    /* no errors */
    *ppLinkParms = &pLinkParmsAdapter->linkParms;
    dwRetVal = EC_E_NOERROR;

Exit:
    if (EC_E_NOERROR != dwRetVal)
    {
        SafeOsFree(pLinkParmsAdapter);
    }
    return dwRetVal;
}
#endif /* INCLUDE_EMLLALTERATSE */

#if (defined INCLUDE_EMLL_SOC_BCMGENET)
/***************************************************************************************************/
/**
\brief  Create BcmGenet link layer parameters according to current command line parsing

\return EC_E_NOERROR     if link layer parameters was created
        EC_E_NOTFOUND    if command line was not matching
        EC_E_INVALIDPARM if syntax error
*/
static EC_T_DWORD CreateLinkParmsFromCmdLineBcmGenet(EC_T_CHAR** ptcWord, EC_T_CHAR** lpCmdLine, EC_T_CHAR* tcStorage, EC_T_BOOL* pbGetNextWord,
    EC_T_LINK_PARMS** ppLinkParms)
{
    EC_T_DWORD dwRetVal = EC_E_ERROR;
    EC_T_LINK_PARMS_BCMGENET* pLinkParmsAdapter = EC_NULL;

    EC_UNREFPARM(pbGetNextWord);

    /* check for matching adapter */
    if (OsStricmp(*ptcWord, "-bcmgenet") != 0)
    {
        dwRetVal = EC_E_NOTFOUND;
        goto Exit;
    }
    /* alloc adapter specific link parms */
    pLinkParmsAdapter = (EC_T_LINK_PARMS_BCMGENET*)OsMalloc(sizeof(EC_T_LINK_PARMS_BCMGENET));
    if (EC_NULL == pLinkParmsAdapter)
    {
        dwRetVal = EC_E_NOMEMORY;
        goto Exit;
    }
    OsMemset(pLinkParmsAdapter, 0, sizeof(EC_T_LINK_PARMS_BCMGENET));
    LinkParmsInit(&pLinkParmsAdapter->linkParms, EC_LINK_PARMS_SIGNATURE_BCMGENET, sizeof(EC_T_LINK_PARMS_BCMGENET), EC_LINK_PARMS_IDENT_BCMGENET, 1, EcLinkMode_POLLING);

    /* parse mandatory parameters: instance, mode */
    if (!ParseDword(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.dwInstance)
        || !ParseLinkMode(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.eLinkMode))
    {
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }
    /* no errors */
    *ppLinkParms = &pLinkParmsAdapter->linkParms;
    dwRetVal = EC_E_NOERROR;

Exit:
    if (EC_E_NOERROR != dwRetVal)
    {
        SafeOsFree(pLinkParmsAdapter);
    }
    return dwRetVal;
}
#endif /* INCLUDE_EMLL_SOC_BCMGENET */

#if (defined INCLUDE_EMLLCCAT)
/***************************************************************************************************/
/**
\brief  Create CCAT link layer parameters according to current command line parsing

\return EC_E_NOERROR     if link layer parameters was created
        EC_E_NOTFOUND    if command line was not matching
        EC_E_INVALIDPARM if syntax error
*/
static EC_T_DWORD CreateLinkParmsFromCmdLineCCAT(EC_T_CHAR** ptcWord, EC_T_CHAR** lpCmdLine, EC_T_CHAR* tcStorage, EC_T_BOOL* pbGetNextWord,
                                                 EC_T_LINK_PARMS** ppLinkParms)
{
EC_T_DWORD dwRetVal = EC_E_ERROR;
EC_T_LINK_PARMS_CCAT* pLinkParmsAdapter = EC_NULL;

    EC_UNREFPARM(pbGetNextWord);

    /* check for matching adapter */
    if (OsStricmp(*ptcWord, "-ccat") != 0)
    {
        dwRetVal = EC_E_NOTFOUND;
        goto Exit;
    }
    /* alloc adapter specific link parms */
    pLinkParmsAdapter = (EC_T_LINK_PARMS_CCAT*)OsMalloc(sizeof(EC_T_LINK_PARMS_CCAT));
    if (EC_NULL == pLinkParmsAdapter)
    {
        dwRetVal = EC_E_NOMEMORY;
        goto Exit;
    }
    OsMemset(pLinkParmsAdapter, 0, sizeof(EC_T_LINK_PARMS_CCAT));
    LinkParmsInit(&pLinkParmsAdapter->linkParms, EC_LINK_PARMS_SIGNATURE_CCAT, sizeof(EC_T_LINK_PARMS_CCAT), EC_LINK_PARMS_IDENT_CCAT, 1, EcLinkMode_POLLING);

    /* parse mandatory parameters: instance, mode */
    if ( !ParseDword(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.dwInstance)
        || !ParseLinkMode(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.eLinkMode))
    {
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }

    /* PCI / EIM */
#if (EC_ARCH == EC_ARCH_ARM)
    pLinkParmsAdapter->eCcatType = eCCAT_EIM;
    pLinkParmsAdapter->qwCcatBase = 0xf0000000;
    pLinkParmsAdapter->dwCcatSize = 0x02000000;
    pLinkParmsAdapter->dwRxBufferCnt = 20;
    pLinkParmsAdapter->dwTxBufferCnt = 20;
#else
    pLinkParmsAdapter->eCcatType = eCCAT_PCI;
#endif

    /* no errors */
    *ppLinkParms = &pLinkParmsAdapter->linkParms;
    dwRetVal = EC_E_NOERROR;

Exit:
    if (EC_E_NOERROR != dwRetVal)
    {
        SafeOsFree(pLinkParmsAdapter);
    }
    return dwRetVal;
}
#endif /* INCLUDE_EMLLCCAT */

#if (defined INCLUDE_EMLLCPSW)
/***************************************************************************************************/
/**
\brief  Parses EC_T_CPSW_TYPE parameter value from command line

\return EC_TRUE if successfully parsed, EC_FALSE on syntax errors
*/
EC_T_BOOL ParseCPSWType(
    EC_T_CHAR**     ptcWord,
    EC_T_CHAR**     lpCmdLine,
    EC_T_CHAR*      tcStorage,
    EC_T_CPSW_TYPE*  peType)
{
    EC_T_BOOL bRes = EC_FALSE;

    if (ParseString(ptcWord, lpCmdLine, tcStorage))
    {
        bRes = EC_TRUE;
        if (0 == OsStricmp(*ptcWord, "am33XX"))       { *peType = eCPSW_AM33XX; }
        else if (0 == OsStricmp(*ptcWord, "am437X"))  { *peType = eCPSW_AM437X; }
        else if (0 == OsStricmp(*ptcWord, "am57X"))   { *peType = eCPSW_AM57X;  }
        else if (0 == OsStricmp(*ptcWord, "am387X"))  { *peType = eCPSW_AM387X; }
        else
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Invalid CPSW type (type == %s)\n", ptcWord));
            bRes = EC_FALSE;
        }
    }
    return bRes;
}

/***************************************************************************************************/
/**
\brief  Create CPSW link layer parameters according to current command line parsing

\return EC_E_NOERROR     if link layer parameters was created
        EC_E_NOTFOUND    if command line was not matching
        EC_E_INVALIDPARM if syntax error
*/
static EC_T_DWORD CreateLinkParmsFromCmdLineCPSW(EC_T_CHAR** ptcWord, EC_T_CHAR** lpCmdLine, EC_T_CHAR* tcStorage, EC_T_BOOL* pbGetNextWord,
                                                 EC_T_LINK_PARMS** ppLinkParms)
{
EC_T_DWORD dwRetVal = EC_E_ERROR;
EC_T_LINK_PARMS_CPSW* pLinkParmsAdapter = EC_NULL;

    /* check for matching adapter */
    if (OsStricmp(*ptcWord, "-cpsw") != 0)
    {
        dwRetVal = EC_E_NOTFOUND;
        goto Exit;
    }
    /* alloc adapter specific link parms */
    pLinkParmsAdapter = (EC_T_LINK_PARMS_CPSW*)OsMalloc(sizeof(EC_T_LINK_PARMS_CPSW));
    if (EC_NULL == pLinkParmsAdapter)
    {
        dwRetVal = EC_E_NOMEMORY;
        goto Exit;
    }
    OsMemset(pLinkParmsAdapter, 0, sizeof(EC_T_LINK_PARMS_CPSW));
    LinkParmsInit(&pLinkParmsAdapter->linkParms, EC_LINK_PARMS_SIGNATURE_CPSW, sizeof(EC_T_LINK_PARMS_CPSW), EC_LINK_PARMS_IDENT_CPSW, 1, EcLinkMode_POLLING);

    /* parse mandatory parameters: port (instance), mode, port priority, Master flag*/
    if ( !ParseDword(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.dwInstance)
        || !ParseLinkMode(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.eLinkMode)
        || !ParseDword(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->dwPortPrio)
        || /* parse bMaster */ !ParseString(ptcWord, lpCmdLine, tcStorage))
    {
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }
    pLinkParmsAdapter->bMaster = (*ptcWord[0] == 'm');

    /* set default parameters */
    pLinkParmsAdapter->bNotUseDmaBuffers = EC_TRUE;
    pLinkParmsAdapter->ePhyInterface = ePHY_GMII;
    pLinkParmsAdapter->eCpswType = eCPSW_AM33XX;
    pLinkParmsAdapter->dwPhyAddr = pLinkParmsAdapter->linkParms.dwInstance - 1; /* 0 -> Port1, 1 -> Port2 */

    /* parse optional parameters  */

    /* get reference board */
    if (!ParseString(ptcWord, lpCmdLine, tcStorage))
    {
        if (EC_NULL != pbGetNextWord)
        {
            *pbGetNextWord = EC_FALSE;
        }
    }
    else
    {
        if (0 == OsStricmp(*ptcWord, "bone"))
        {
            pLinkParmsAdapter->eCpswType = eCPSW_AM33XX;
        }
        else if (0 == OsStricmp(*ptcWord, "am3359-icev2"))
        {
            pLinkParmsAdapter->eCpswType = eCPSW_AM33XX;
            pLinkParmsAdapter->ePhyInterface = ePHY_RGMII;
            if (1 == pLinkParmsAdapter->linkParms.dwInstance)
            {
                pLinkParmsAdapter->dwPhyAddr = 1;
            }
            else
            {
                pLinkParmsAdapter->dwPhyAddr = 3;
            }
        }
        else if (0 == OsStricmp(*ptcWord, "am437x-idk"))
        {
            pLinkParmsAdapter->eCpswType = eCPSW_AM437X;
        }
        else if (0 == OsStricmp(*ptcWord, "am572x-idk"))
        {
            pLinkParmsAdapter->eCpswType = eCPSW_AM57X;
            pLinkParmsAdapter->ePhyInterface = ePHY_RGMII;
        }
        else if (0 == OsStricmp(*ptcWord, "387X_evm"))
        {
            pLinkParmsAdapter->eCpswType = eCPSW_AM387X;
        }
        else if (0 == OsStricmp(*ptcWord, "custom"))
        {
            EC_T_DWORD dwNotUseDmaBuffers = 0;
            /* parse CpswType, PHY address, PHY connection type, use DMA */
            if (!ParseCPSWType(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->eCpswType)
                || !ParseDword(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->dwPhyAddr))
            {
                EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "error parsing CPSW parameters for custom board\n"));
                dwRetVal = EC_E_INVALIDPARM;
                goto Exit;
            }
            /* get PHY interface */
            *ptcWord = GetNextWord(lpCmdLine, tcStorage);
            if      (0 == OsStricmp(*ptcWord, "rmii"))  pLinkParmsAdapter->ePhyInterface = ePHY_RMII;
            else if (0 == OsStricmp(*ptcWord, "gmii"))  pLinkParmsAdapter->ePhyInterface = ePHY_GMII;
            else if (0 == OsStricmp(*ptcWord, "rgmii")) pLinkParmsAdapter->ePhyInterface = ePHY_RGMII;
            else if (0 == OsStricmp(*ptcWord, "osdriver")) pLinkParmsAdapter->ePhyInterface = ePHY_OSDRIVER;
            else 
            {
                EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "error parsing CPSW parameters for custom board\n"));
                dwRetVal = EC_E_INVALIDPARM;
                goto Exit;
            }
            if (!ParseDword(ptcWord, lpCmdLine, tcStorage, &dwNotUseDmaBuffers))
            {
                EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "error parsing CPSW parameters for custom board\n"));
                dwRetVal = EC_E_INVALIDPARM;
                goto Exit;
            }

            pLinkParmsAdapter->bNotUseDmaBuffers = (dwNotUseDmaBuffers == 0) ? EC_FALSE : EC_TRUE;
        }
        else
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Invalid RefBoard value\n"));
            dwRetVal = EC_E_INVALIDPARM;
            goto Exit;
        }
    }

    /* verify parameter values */
    if (pLinkParmsAdapter->linkParms.dwInstance < 1 || pLinkParmsAdapter->linkParms.dwInstance > 2)
    {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Port number must be 1 or 2\n"));
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }
    if (pLinkParmsAdapter->dwPortPrio != 0 && pLinkParmsAdapter->dwPortPrio != 1)
    {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Port priority must be 0 or 1\n"));
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }

    pLinkParmsAdapter->bPhyRestartAutoNegotiation = EC_TRUE;

    /* no errors */
    *ppLinkParms = &pLinkParmsAdapter->linkParms;
    dwRetVal = EC_E_NOERROR;

Exit:
    if (EC_E_NOERROR != dwRetVal)
    {
        SafeOsFree(pLinkParmsAdapter);
    }
    return dwRetVal;
}
#endif /* INCLUDE_EMLLCPSW */

#if (defined INCLUDE_EMLLDUMMY)
/***************************************************************************************************/
/**
\brief  Create dummy link layer parameters according to current command line parsing

\return EC_E_NOERROR     if link layer parameters was created
EC_E_NOTFOUND    if command line was not matching
EC_E_INVALIDPARM if syntax error
*/
static EC_T_DWORD CreateLinkParmsFromCmdLineDummy(EC_T_CHAR** ptcWord, EC_T_CHAR** lpCmdLine, EC_T_CHAR* tcStorage, EC_T_BOOL* pbGetNextWord,
    EC_T_LINK_PARMS** ppLinkParms)
{
    EC_T_DWORD dwRetVal = EC_E_ERROR;
    EC_T_LINK_PARMS_DUMMY* pLinkParmsAdapter = EC_NULL;

    EC_UNREFPARM(pbGetNextWord);

    /* check for matching adapter */
    if (OsStricmp(*ptcWord, "-dummy") != 0)
    {
        dwRetVal = EC_E_NOTFOUND;
        goto Exit;
    }
    /* alloc adapter specific link parms */
    pLinkParmsAdapter = (EC_T_LINK_PARMS_DUMMY*)OsMalloc(sizeof(EC_T_LINK_PARMS_DUMMY));
    if (EC_NULL == pLinkParmsAdapter)
    {
        dwRetVal = EC_E_NOMEMORY;
        goto Exit;
    }
    OsMemset(pLinkParmsAdapter, 0, sizeof(EC_T_LINK_PARMS_DUMMY));
    LinkParmsInit(&pLinkParmsAdapter->linkParms, EC_LINK_PARMS_SIGNATURE_DUMMY, sizeof(EC_T_LINK_PARMS_DUMMY), EC_LINK_PARMS_IDENT_DUMMY, 1, EcLinkMode_POLLING);

    /* get Instance and Mode */
    if (!ParseDword(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.dwInstance)
        || !ParseLinkMode(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.eLinkMode))
    {
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }

    /* no errors */
    *ppLinkParms = &pLinkParmsAdapter->linkParms;
    dwRetVal = EC_E_NOERROR;

Exit:
    if (EC_E_NOERROR != dwRetVal)
    {
        SafeOsFree(pLinkParmsAdapter);
    }
    return dwRetVal;
}
#endif /* INCLUDE_EMLLDUMMY */

#if (defined INCLUDE_EMLLDW3504)
/***************************************************************************************************/
/**
\brief  Create DW3504 link layer parameters according to current command line parsing

\return EC_E_NOERROR     if link layer parameters was created
EC_E_NOTFOUND    if command line was not matching
EC_E_INVALIDPARM if syntax error
*/
static EC_T_DWORD CreateLinkParmsFromCmdLineDW3504(EC_T_CHAR** ptcWord, EC_T_CHAR** lpCmdLine, EC_T_CHAR* tcStorage, EC_T_BOOL* pbGetNextWord,
    EC_T_LINK_PARMS** ppLinkParms)
{
    EC_T_DWORD dwRetVal = EC_E_ERROR;
    EC_T_LINK_PARMS_DW3504* pLinkParmsAdapter = EC_NULL;

    EC_UNREFPARM(pbGetNextWord);

    /* check for matching adapter */
    if (OsStricmp(*ptcWord, "-DW3504") != 0)
    {
        dwRetVal = EC_E_NOTFOUND;
        goto Exit;
    }
    /* alloc adapter specific link parms */
    pLinkParmsAdapter = (EC_T_LINK_PARMS_DW3504*)OsMalloc(sizeof(EC_T_LINK_PARMS_DW3504));
    if (EC_NULL == pLinkParmsAdapter)
    {
        dwRetVal = EC_E_NOMEMORY;
        goto Exit;
    }
    OsMemset(pLinkParmsAdapter, 0, sizeof(EC_T_LINK_PARMS_DW3504));
    LinkParmsInit(&pLinkParmsAdapter->linkParms, EC_LINK_PARMS_SIGNATURE_DW3504, sizeof(EC_T_LINK_PARMS_DW3504), EC_LINK_PARMS_IDENT_DW3504, 1, EcLinkMode_POLLING);

    /* parse mandatory parameters */
    if (!ParseDword(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.dwInstance)
        || !ParseLinkMode(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.eLinkMode))
    {
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }

    /* get reference board */
    if (!ParseString(ptcWord, lpCmdLine, tcStorage))
    {
        if (EC_NULL != pbGetNextWord)
        {
            *pbGetNextWord = EC_FALSE;
        }

        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }
    else
    {
        if (0 == OsStricmp(*ptcWord, "socrates"))
        {
            pLinkParmsAdapter->eDW3504Type = eDW3504_CycloneV;
            pLinkParmsAdapter->ePhyInterface = ePHY_MII;
            if (pLinkParmsAdapter->linkParms.dwInstance == 1)
            {
                pLinkParmsAdapter->dwRegisterBasePhys = 0xFF700000;
                pLinkParmsAdapter->dwPhyAddr = 0;
            }
            else if (pLinkParmsAdapter->linkParms.dwInstance == 2)
            {
                pLinkParmsAdapter->dwRegisterBasePhys = 0xFF702000;
                pLinkParmsAdapter->dwPhyAddr = 1;
            }
            else
            {
                EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Instance number must be 1 or 2\n"));
                dwRetVal = EC_E_INVALIDPARM;
                goto Exit;
            }
        }
        else if (0 == OsStricmp(*ptcWord, "rd55up06"))
        {
            pLinkParmsAdapter->eDW3504Type = eDW3504_CycloneV;
            pLinkParmsAdapter->ePhyInterface = ePHY_SGMII;
            if (pLinkParmsAdapter->linkParms.dwInstance == 2)
            {
                pLinkParmsAdapter->dwRegisterBasePhys = 0xFF702000;
                pLinkParmsAdapter->dwPhyAddr = 0;
            }
            else
            {
                EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Instance number must be 2\n"));
                dwRetVal = EC_E_INVALIDPARM;
                goto Exit;
            }
        }
        else if (0 == OsStricmp(*ptcWord, "r12ccpu"))
        {
            pLinkParmsAdapter->eDW3504Type = eDW3504_CycloneV;
            if (pLinkParmsAdapter->linkParms.dwInstance == 1)
            {
                pLinkParmsAdapter->ePhyInterface = ePHY_GMII;
                pLinkParmsAdapter->dwRegisterBasePhys = 0xFF700000;
                pLinkParmsAdapter->dwPhyAddr = 0;
            }
            else if (pLinkParmsAdapter->linkParms.dwInstance == 2)
            {
                pLinkParmsAdapter->ePhyInterface = ePHY_SGMII;
                pLinkParmsAdapter->dwRegisterBasePhys = 0xFF702000;
                pLinkParmsAdapter->dwPhyAddr = 0;
            }
            else
            {
                EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Instance number must be 1 or 2\n"));
                dwRetVal = EC_E_INVALIDPARM;
                goto Exit;
            }
        }
        else if (0 == OsStricmp(*ptcWord, "lces1"))
        {
            pLinkParmsAdapter->eDW3504Type = eDW3504_LCES1;
            pLinkParmsAdapter->ePhyInterface = ePHY_MII;
            if (pLinkParmsAdapter->linkParms.dwInstance == 1)
            {
                pLinkParmsAdapter->dwRegisterBasePhys = 0x44000000;
                pLinkParmsAdapter->dwPhyAddr = 0;
            }
            else if (pLinkParmsAdapter->linkParms.dwInstance == 2)
            {
                pLinkParmsAdapter->dwRegisterBasePhys = 0x44002000;
                pLinkParmsAdapter->dwPhyAddr = 4;
            }
            else
            {
                EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Instance number must be 1 or 2\n"));
                dwRetVal = EC_E_INVALIDPARM;
                goto Exit;
            }
        }
        else if (0 == OsStricmp(*ptcWord, "rzn1"))
        {
            pLinkParmsAdapter->eDW3504Type = eDW3504_RZN1;
            pLinkParmsAdapter->ePhyInterface = ePHY_RGMII;
            if (pLinkParmsAdapter->linkParms.dwInstance == 1)
            {
                pLinkParmsAdapter->dwRegisterBasePhys = 0x44000000;
                pLinkParmsAdapter->dwPhyAddr = 8;
            }
            else if (pLinkParmsAdapter->linkParms.dwInstance == 2)
            {
                pLinkParmsAdapter->dwRegisterBasePhys = 0x44002000;
            }
            else
            {
                EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Instance number must be 1 or 2\n"));
                dwRetVal = EC_E_INVALIDPARM;
                goto Exit;
            }
        }
        else if (0 == OsStricmp(*ptcWord, "stm32mp157a-dk1"))
        {
            pLinkParmsAdapter->eDW3504Type = eDW3504_STM32MP1;
            pLinkParmsAdapter->ePhyInterface = ePHY_RGMII;
            if (pLinkParmsAdapter->linkParms.dwInstance == 1)
            {
                pLinkParmsAdapter->dwRegisterBasePhys = 0x5800A000;
                pLinkParmsAdapter->dwPhyAddr = 0;
            }
            else
            {
                EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Instance number must be 1\n"));
                dwRetVal = EC_E_INVALIDPARM;
                goto Exit;
            }
        }
        else if (0 == OsStricmp(*ptcWord, "intel_atom"))
        {
            pLinkParmsAdapter->eDW3504Type = eDW3504_ATOM;
            pLinkParmsAdapter->ePhyInterface = ePHY_UNDEFINED;
            pLinkParmsAdapter->dwPhyAddr = 1;
            pLinkParmsAdapter->dwRegisterBasePhys = 0;
        }
        else if (0 == OsStricmp(*ptcWord, "custom"))
        {
            /* get DW3504 type */
            *ptcWord = GetNextWord(lpCmdLine, tcStorage);
            if (0 == OsStricmp(*ptcWord, "cycloneV"))
            {
                pLinkParmsAdapter->eDW3504Type = eDW3504_CycloneV;
                if (pLinkParmsAdapter->linkParms.dwInstance == 1)
                {
                    pLinkParmsAdapter->dwRegisterBasePhys = 0xFF700000;
                }
                else if (pLinkParmsAdapter->linkParms.dwInstance == 2)
                {
                    pLinkParmsAdapter->dwRegisterBasePhys = 0xFF702000;
                }
                else
                {
                    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Instance number must be 1 or 2\n"));
                    dwRetVal = EC_E_INVALIDPARM;
                    goto Exit;
                }
            }
            else if (0 == OsStricmp(*ptcWord, "lces1"))
            {
                pLinkParmsAdapter->eDW3504Type = eDW3504_LCES1;
                if (pLinkParmsAdapter->linkParms.dwInstance == 1)
                {
                    pLinkParmsAdapter->dwRegisterBasePhys = 0x44000000;
                }
                else if (pLinkParmsAdapter->linkParms.dwInstance == 2)
                {
                    pLinkParmsAdapter->dwRegisterBasePhys = 0x44002000;
                }
                else
                {
                    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Instance number must be 1 or 2\n"));
                    dwRetVal = EC_E_INVALIDPARM;
                    goto Exit;
                }
            }
            else if (0 == OsStricmp(*ptcWord, "stm32mp157a-dk1"))
            {
                pLinkParmsAdapter->eDW3504Type = eDW3504_STM32MP1;
                pLinkParmsAdapter->ePhyInterface = ePHY_OSDRIVER;
                if (pLinkParmsAdapter->linkParms.dwInstance == 1)
                {
                    pLinkParmsAdapter->dwRegisterBasePhys = 0x5800A000;
                    pLinkParmsAdapter->dwPhyAddr = 1;
                }
                else
                {
                    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Instance number must be 1\n"));
                    dwRetVal = EC_E_INVALIDPARM;
                    goto Exit;
                }
            }
            else if (0 == OsStricmp(*ptcWord, "intel_atom"))
            {
                pLinkParmsAdapter->eDW3504Type = eDW3504_ATOM;
                pLinkParmsAdapter->dwPhyAddr = 1;
                pLinkParmsAdapter->dwRegisterBasePhys = 0;
                pLinkParmsAdapter->ePhyInterface = ePHY_UNDEFINED;
            }
            else
            {
                EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Invalid eDW3504Type value\n"));
                dwRetVal = EC_E_INVALIDPARM;
                goto Exit;
            }
            /* get PHY interface */
            *ptcWord = GetNextWord(lpCmdLine, tcStorage);
            if (0 == OsStricmp(*ptcWord, "fixed")) pLinkParmsAdapter->ePhyInterface = ePHY_FIXED_LINK;
            else if (0 == OsStricmp(*ptcWord, "mii"))   pLinkParmsAdapter->ePhyInterface = ePHY_MII;
            else if (0 == OsStricmp(*ptcWord, "rmii"))  pLinkParmsAdapter->ePhyInterface = ePHY_RMII;
            else if (0 == OsStricmp(*ptcWord, "gmii"))  pLinkParmsAdapter->ePhyInterface = ePHY_GMII;
            else if (0 == OsStricmp(*ptcWord, "sgmii")) pLinkParmsAdapter->ePhyInterface = ePHY_SGMII;
            else if (0 == OsStricmp(*ptcWord, "rgmii")) pLinkParmsAdapter->ePhyInterface = ePHY_RGMII;
            else if (0 == OsStricmp(*ptcWord, "osdriver"))  pLinkParmsAdapter->ePhyInterface = ePHY_OSDRIVER;
            else if (0 == OsStricmp(*ptcWord, "undefined"))  pLinkParmsAdapter->ePhyInterface = ePHY_UNDEFINED;
            else
            {
                EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Invalid PhyInterface value\n"));
                dwRetVal = EC_E_INVALIDPARM;
                goto Exit;
            }
            /* get PHY address */
            if (!ParseDword(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->dwPhyAddr))
            {
                pLinkParmsAdapter->dwPhyAddr = 0;
            }
        }
        else
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Invalid RefBoard value\n"));
            dwRetVal = EC_E_INVALIDPARM;
            goto Exit;
        }
    }
    /* set default values */
#if (defined __arm__) || (defined __ICCARM__)
    pLinkParmsAdapter->bNotUseDmaBuffers = EC_TRUE;
#else
    pLinkParmsAdapter->bNotUseDmaBuffers = EC_FALSE;
#endif

    /* no errors */
    *ppLinkParms = &pLinkParmsAdapter->linkParms;
    dwRetVal = EC_E_NOERROR;

Exit:
    if (EC_E_NOERROR != dwRetVal)
    {
        SafeOsFree(pLinkParmsAdapter);
    }
    return dwRetVal;
}
#endif /* INCLUDE_EMLLDW3504 */

#if (defined INCLUDE_EMLLEG20T)
/***************************************************************************************************/
/**
\brief  Create EG20T link layer parameters according to current command line parsing

\return EC_E_NOERROR     if link layer parameters was created
EC_E_NOTFOUND    if command line was not matching
EC_E_INVALIDPARM if syntax error
*/
static EC_T_DWORD CreateLinkParmsFromCmdLineEG20T(EC_T_CHAR** ptcWord, EC_T_CHAR** lpCmdLine, EC_T_CHAR* tcStorage, EC_T_BOOL* pbGetNextWord,
    EC_T_LINK_PARMS** ppLinkParms)
{
    EC_T_DWORD dwRetVal = EC_E_ERROR;
    EC_T_LINK_PARMS_EG20T* pLinkParmsAdapter = EC_NULL;

    EC_UNREFPARM(pbGetNextWord);

    /* check for matching adapter */
    if (OsStricmp(*ptcWord, "-eg20t") != 0)
    {
        dwRetVal = EC_E_NOTFOUND;
        goto Exit;
    }
    /* alloc adapter specific link parms */
    pLinkParmsAdapter = (EC_T_LINK_PARMS_EG20T*)OsMalloc(sizeof(EC_T_LINK_PARMS_EG20T));
    if (EC_NULL == pLinkParmsAdapter)
    {
        dwRetVal = EC_E_NOMEMORY;
        goto Exit;
    }
    OsMemset(pLinkParmsAdapter, 0, sizeof(EC_T_LINK_PARMS_EG20T));
    LinkParmsInit(&pLinkParmsAdapter->linkParms, EC_LINK_PARMS_SIGNATURE_EG20T, sizeof(EC_T_LINK_PARMS_EG20T), EC_LINK_PARMS_IDENT_EG20T, 1, EcLinkMode_POLLING);

    /* parse mandatory parameters: instance, mode */
    if (!ParseDword(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.dwInstance)
        || !ParseLinkMode(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.eLinkMode))
    {
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }
    /* no errors */
    *ppLinkParms = &pLinkParmsAdapter->linkParms;
    dwRetVal = EC_E_NOERROR;

Exit:
    if (EC_E_NOERROR != dwRetVal)
    {
        SafeOsFree(pLinkParmsAdapter);
    }
    return dwRetVal;
}
#endif /* INCLUDE_EMLLEG20T */

#if (defined INCLUDE_EMLLEMAC)
/***************************************************************************************************/
/**
\brief  Create EMAC link layer parameters according current command line parsing

\return EC_E_NOERROR     if link layer parameters was created
        EC_E_NOTFOUND    if command line was not matching
        EC_E_INVALIDPARM if syntax error
*/
static EC_T_DWORD CreateLinkParmsFromCmdLineEMAC(EC_T_CHAR** ptcWord, EC_T_CHAR** lpCmdLine, EC_T_CHAR* tcStorage, EC_T_BOOL* pbGetNextWord,
                                                 EC_T_LINK_PARMS** ppLinkParms)
{
EC_T_DWORD dwRetVal = EC_E_ERROR;
EC_T_LINK_PARMS_EMAC* pLinkParmsAdapter = EC_NULL;

    /* check for matching adapter */
    if (OsStricmp(*ptcWord, "-emac") != 0)
    {
        dwRetVal = EC_E_NOTFOUND;
        goto Exit;
    }
    /* alloc adapter specific link parms */
    pLinkParmsAdapter = (EC_T_LINK_PARMS_EMAC*)OsMalloc(sizeof(EC_T_LINK_PARMS_EMAC));
    if (EC_NULL == pLinkParmsAdapter)
    {
        dwRetVal = EC_E_NOMEMORY;
        goto Exit;
    }
    OsMemset(pLinkParmsAdapter, 0, sizeof(EC_T_LINK_PARMS_EMAC));
    LinkParmsInit(&pLinkParmsAdapter->linkParms, EC_LINK_PARMS_SIGNATURE_EMAC, sizeof(EC_T_LINK_PARMS_EMAC), EC_LINK_PARMS_IDENT_EMAC, 1, EcLinkMode_POLLING);

    /* parse mandatory parameters: instance, mode */
    if ( !ParseDword(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.dwInstance)
        || !ParseLinkMode(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.eLinkMode))
    {
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }

    /* default parameters */
    pLinkParmsAdapter->dwRegisterBase = 0x86000000;

    /* get reference board */
    if (!ParseString(ptcWord, lpCmdLine, tcStorage))
    {
        if (EC_NULL != pbGetNextWord)
        {
            *pbGetNextWord = EC_FALSE;
        }
    }
    else
    {
        if(0 == OsStricmp(*ptcWord, "MC2002E"))
        {
            pLinkParmsAdapter->dwRegisterBase   = 0x80000000;
            pLinkParmsAdapter->dwRegisterLength = 0x2000;
            pLinkParmsAdapter->bNotUseDmaBuffers = EC_TRUE;
        }
        else if (0 == OsStricmp(*ptcWord, "custom"))
        {
            /* get register base */
            *ptcWord = GetNextWord(lpCmdLine, tcStorage);
            if (EC_NULL != ptcWord)
            {
                pLinkParmsAdapter->dwRegisterBase = (EC_T_DWORD)OsStrtoul(*ptcWord,0,16);
            }
            else
            {
                EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Missing parameter RegisterBase\n"));
                dwRetVal = EC_E_INVALIDPARM;
                goto Exit;
            }
            /* get register length */
            *ptcWord = GetNextWord(lpCmdLine, tcStorage);
            if (EC_NULL != ptcWord)
            {
                pLinkParmsAdapter->dwRegisterLength = (EC_T_DWORD)OsStrtoul(*ptcWord,0,16);
            }
            else
            {
                EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Missing parameter RegisterLength\n"));
                dwRetVal = EC_E_INVALIDPARM;
                goto Exit;
            }
            /* use dma buffers */
            *ptcWord = GetNextWord(lpCmdLine, tcStorage);
            if (EC_NULL != ptcWord)
            {
                pLinkParmsAdapter->bNotUseDmaBuffers = (EC_T_BOOL)OsStrtoul(*ptcWord,0,16);
            }
            else
            {
                EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Missing parameter NotUseDmaBuffers\n"));
                dwRetVal = EC_E_INVALIDPARM;
                goto Exit;
            }
        }
        else
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Invalid RefBoard value\n"));
            dwRetVal = EC_E_INVALIDPARM;
            goto Exit;
        }
    }
    pLinkParmsAdapter->dwPhyAddr        = 0;
    pLinkParmsAdapter->dwRxInterrupt    = 1; /* from xparameters.h */

    /* no errors */
    *ppLinkParms = &pLinkParmsAdapter->linkParms;
    dwRetVal = EC_E_NOERROR;

Exit:
    if (EC_E_NOERROR != dwRetVal)
    {
        SafeOsFree(pLinkParmsAdapter);
    }
    return dwRetVal;
}
#endif /* INCLUDE_EMLLEMAC */

#if (defined INCLUDE_EMLLETSEC)
#if (defined EC_VERSION_VXWORKS)
extern "C" {
UINT32 sysGetPeripheralBase (void); /* from sysLib.c */
}
#endif

/***************************************************************************************************/
/**
\brief  Create ETSEC link layer parameters according to current command line parsing

\return EC_E_NOERROR     if link layer parameters was created
        EC_E_NOTFOUND    if command line was not matching
        EC_E_INVALIDPARM if syntax error
*/
static EC_T_DWORD CreateLinkParmsFromCmdLineETSEC(EC_T_CHAR** ptcWord, EC_T_CHAR** lpCmdLine, EC_T_CHAR* tcStorage, EC_T_BOOL* pbGetNextWord,
                                                  EC_T_LINK_PARMS** ppLinkParms)
{
    EC_T_DWORD dwRetVal = EC_E_ERROR;
    EC_T_LINK_PARMS_ETSEC* pLinkParmsAdapter = EC_NULL;
    EC_T_DWORD dwCcsrbar = 0x0;
    EC_T_BOOL  bCustom = EC_FALSE;
    EC_T_DWORD dwCustomPhyAddr = 0;
    EC_T_DWORD dwCustomRxIrq = 0;
    EC_T_DWORD bCustomNotUseDmaBuffers = EC_FALSE;

    EC_UNREFPARM(pbGetNextWord);

    /* check for matching adapter */
    if (OsStricmp(*ptcWord, "-fsletsec") != 0)
    {
        dwRetVal = EC_E_NOTFOUND;
        goto Exit;
    }
    /* alloc adapter specific link parms */
    pLinkParmsAdapter = (EC_T_LINK_PARMS_ETSEC*)OsMalloc(sizeof(EC_T_LINK_PARMS_ETSEC));
    if (EC_NULL == pLinkParmsAdapter)
    {
        dwRetVal = EC_E_NOMEMORY;
        goto Exit;
    }
    OsMemset(pLinkParmsAdapter, 0, sizeof(EC_T_LINK_PARMS_ETSEC));
    LinkParmsInit(&pLinkParmsAdapter->linkParms, EC_LINK_PARMS_SIGNATURE_ETSEC, sizeof(EC_T_LINK_PARMS_ETSEC), EC_LINK_PARMS_IDENT_ETSEC, 1, EcLinkMode_POLLING);

    /* parse mandatory parameters: instance, mode */
    if ( !ParseDword(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.dwInstance)
        || !ParseLinkMode(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.eLinkMode))
    {
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }

    /* get reference board */
    *ptcWord = GetNextWord(lpCmdLine, tcStorage);
    if (0 == OsStricmp(*ptcWord, "p2020rdb"))
    {
        pLinkParmsAdapter->eETSECType = eETSEC_P2020RDB;
    }
    else if (0 == OsStricmp(*ptcWord, "twrp1025"))
    {
        pLinkParmsAdapter->eETSECType = eETSEC_TWRP1025;
    }
#if (defined EC_VERSION_VXWORKS)
    else if (0 == OsStricmp(*ptcWord, "istmpc8548"))
    {
        pLinkParmsAdapter->eETSECType = eETSEC_ISTMPC8548;
    }
    else if (0 == OsStricmp(*ptcWord, "xj_epu20c"))
    {
        pLinkParmsAdapter->eETSECType = eETSEC_XJ_EPU20C;
    }
#endif
    else if (0 == OsStricmp(*ptcWord, "twrls1021a"))
    {
        pLinkParmsAdapter->eETSECType = eETSEC_TWRLS1021A;
    }
    else if (0 == OsStricmp(*ptcWord, "tqmls_ls102xa"))
    {
        pLinkParmsAdapter->eETSECType = eETSEC_TQMLS_LS102XA;
    }
    else if (0 == OsStricmp(*ptcWord, "custom"))
    {
        bCustom = EC_TRUE;

        /* get ETSEC type */
        *ptcWord = GetNextWord(lpCmdLine, tcStorage);
        if (0 == OsStricmp(*ptcWord, "p2020rdb"))
        {
            pLinkParmsAdapter->eETSECType = eETSEC_P2020RDB;
        }
        else if (0 == OsStricmp(*ptcWord, "twrp1025"))
        {
            pLinkParmsAdapter->eETSECType = eETSEC_TWRP1025;
        }
#if (defined EC_VERSION_VXWORKS)
        else if (0 == OsStricmp(*ptcWord, "istmpc8548"))
        {
            pLinkParmsAdapter->eETSECType = eETSEC_ISTMPC8548;
        }
        else if (0 == OsStricmp(*ptcWord, "xj_epu20c"))
        {
            pLinkParmsAdapter->eETSECType = eETSEC_XJ_EPU20C;
        }
#endif
        else if (0 == OsStricmp(*ptcWord, "twrls1021a"))
        {
            pLinkParmsAdapter->eETSECType = eETSEC_TWRLS1021A;
        }
        else if (0 == OsStricmp(*ptcWord, "tqmls_ls102xa"))
        {
            pLinkParmsAdapter->eETSECType = eETSEC_TQMLS_LS102XA;
        }
        else
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Invalid ETSEC type value\n"));
            dwRetVal = EC_E_INVALIDPARM;
            goto Exit;
        }

        /* get PHY address */
        if (!ParseDword(ptcWord, lpCmdLine, tcStorage, &dwCustomPhyAddr))
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Invalid PHY address\n"));
            dwRetVal = EC_E_INVALIDPARM;
            goto Exit;
        }

        /* get Rx IRQ number */
        if (!ParseDword(ptcWord, lpCmdLine, tcStorage, &dwCustomRxIrq))
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Invalid IRQ number\n"));
            dwRetVal = EC_E_INVALIDPARM;
            goto Exit;
        }

        /* get 'bNotUseDmaBuffers' flag */
        if (!ParseDword(ptcWord, lpCmdLine, tcStorage, &bCustomNotUseDmaBuffers))
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Invalid parameter\n"));
            dwRetVal = EC_E_INVALIDPARM;
            goto Exit;
        }
    }
    else
    {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Invalid RefBoard value\n"));
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }

    switch (pLinkParmsAdapter->eETSECType)
    {
    /* Freescale P2020RDB board (CPU P2020E, VxWorks 6.8 PPC / Linux 3.0.9-PREEMPT PPC) */
    case eETSEC_P2020RDB:
        if (pLinkParmsAdapter->linkParms.dwInstance > 3)
        {
           EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Device unit must be <= 3! There are only 3 eTSEC's on P2020RDB.\n"));
           dwRetVal = EC_E_INVALIDPARM;
           goto Exit;
        }

        {
           EC_T_BYTE abyStationAddress[] = {0x00, 0x04, 0x9F, 0x01, 0x79, 0x00};
           memcpy(pLinkParmsAdapter->abyStationAddress, abyStationAddress, 6);
        }

        /* don't change fundamental settings in ETSEC like endianess */
        pLinkParmsAdapter->bMaster = EC_FALSE;

#if (defined EC_VERSION_VXWORKS)
        dwCcsrbar = sysGetPeripheralBase();
#elif (defined EC_VERSION_LINUX)
        dwCcsrbar = 0xffe00000; /* from p2020si.dtsi */

        /* Get interrupt number from Nth eTSEC device in PowerPC device tree */
        pLinkParmsAdapter->dwRxInterrupt = pLinkParmsAdapter->linkParms.dwInstance - 1;
#endif

        pLinkParmsAdapter->dwPhyMdioBase = dwCcsrbar + 0x24000; /* eTSEC 1 */
        pLinkParmsAdapter->dwTbiPhyAddr = 16; /* Dummy address assigned to internal TBI PHY */
        pLinkParmsAdapter->oMiiBusMtx = EC_NULL; /* LinkOsCreateLock(eLockType_DEFAULT); */

        if (pLinkParmsAdapter->linkParms.dwInstance == 1) /* eTSEC1 (4 x 1Gb switchports) */
        {
           pLinkParmsAdapter->abyStationAddress[5] = 0x81;
           pLinkParmsAdapter->dwRegisterBase = dwCcsrbar + 0x24000;
           pLinkParmsAdapter->dwPhyAddr = ETSEC_FIXED_LINK; /* RGMII, No Phy, Switchport */
           pLinkParmsAdapter->dwFixedLinkVal = ETSEC_LINKFLAG_1000baseT_Full | ETSEC_LINKFLAG_LINKOK;
#if (defined EC_VERSION_VXWORKS)
           pLinkParmsAdapter->dwRxInterrupt = 26; /* EPIC_TSEC1RX_INT_VEC from vxbEpicIntCtlr.h */
#endif
        }
        else if (pLinkParmsAdapter->linkParms.dwInstance == 2) /* eTSEC2 */
        {
           pLinkParmsAdapter->abyStationAddress[5] = 0x82;
           pLinkParmsAdapter->dwRegisterBase = dwCcsrbar + 0x25000;
           pLinkParmsAdapter->dwPhyAddr = 0; /* SGMII Phy on addr. 0 (from P2020RDB Ref. Manual) */
#if (defined EC_VERSION_VXWORKS)
           pLinkParmsAdapter->dwRxInterrupt = 32; /* EPIC_TSEC2RX_INT_VEC from vxbEpicIntCtlr.h */
#endif
        }
        else if (pLinkParmsAdapter->linkParms.dwInstance == 3) /* eTSEC3 */
        {
           pLinkParmsAdapter->abyStationAddress[5] = 0x83;
           pLinkParmsAdapter->dwRegisterBase = dwCcsrbar + 0x26000;
           pLinkParmsAdapter->dwPhyAddr = 1; /* RGMII Phy on addr. 1 (from P2020RDB Ref. Manual) */
#if (defined EC_VERSION_VXWORKS)
           pLinkParmsAdapter->dwRxInterrupt = 28; /* EPIC_TSEC3RX_INT_VEC from vxbEpicIntCtlr.h */
#endif
        }

        pLinkParmsAdapter->dwLocalMdioBase = pLinkParmsAdapter->dwRegisterBase;

        break;
    /* Freescale TWR-P1025 board (CPU P1025, Freescale-Linuxkernel 3.0.4)
     * Hardware resource informations (phy-addr, interrupts, io-base, ...)
     * are taken from Linux-kernel's device tree for TWR-P1025 (twr-p1025_32b.dts) */
    case eETSEC_TWRP1025:
        if (pLinkParmsAdapter->linkParms.dwInstance > 2) /* TWR-P1025 has 3 eTSEC's, but only eTSEC1 and eTSEC3 are routed out to RJ45 ports */
        {
           EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Device unit must be <= 2! There are only 2 eTSEC's on P1025TWR.\n"));
           return EC_FALSE;
        }

        {
           EC_T_BYTE abyStationAddress[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xBA, 0xBE };
           memcpy(pLinkParmsAdapter->abyStationAddress, abyStationAddress, sizeof(pLinkParmsAdapter->abyStationAddress));
        }

        pLinkParmsAdapter->bMaster = EC_FALSE;
        dwCcsrbar = 0xffe00000;

        pLinkParmsAdapter->dwPhyMdioBase = dwCcsrbar + 0x24000; /* eTSEC 1, MDIO */
        pLinkParmsAdapter->dwTbiPhyAddr = 16; /* Dummy address assigned to internal TBI PHY */
        pLinkParmsAdapter->oMiiBusMtx = EC_NULL; /* LinkOsCreateLock(eLockType_DEFAULT); */

        /* Get interrupt number from Nth eTSEC device in PowerPC device tree */
        pLinkParmsAdapter->dwRxInterrupt = pLinkParmsAdapter->linkParms.dwInstance - 1;

        if (pLinkParmsAdapter->linkParms.dwInstance == 1) /* eTSEC1 */
        {
           pLinkParmsAdapter->dwRegisterBase = dwCcsrbar + 0xB0000; /* eTSEC1, Group 0 */
           pLinkParmsAdapter->dwLocalMdioBase = dwCcsrbar + 0x24000; /* eTSEC1, MDIO */
           pLinkParmsAdapter->dwPhyAddr = 2;
        }
        else if (pLinkParmsAdapter->linkParms.dwInstance == 2) /* eTSEC3 */
        {
           pLinkParmsAdapter->dwRegisterBase = dwCcsrbar + 0xB2000; /* eTSEC3, Group 0 */
           pLinkParmsAdapter->dwLocalMdioBase = dwCcsrbar + 0x26000; /* eTSEC3, MDIO */
           pLinkParmsAdapter->dwPhyAddr = 1;
        }

        break;

    /* Instron "MPC8548 MiniTower" board (CPU MPC8548, VxWorks 6.9 PPC) */
    case eETSEC_ISTMPC8548:
#if (defined EC_VERSION_VXWORKS)
        if (pLinkParmsAdapter->linkParms.dwInstance > 2) /* MPC8548 has 4 eTSEC's, but only first 2 are routed out to RJ45 ports */
        {
           EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Device unit must be 1 or 2!\n"));
           dwRetVal = EC_E_INVALIDPARM;
           goto Exit;
        }

        {
           EC_T_BYTE abyStationAddress[] = { 0x00, 0x02, 0xCE, 0x90, 0x02, 0x24 };
           memcpy(pLinkParmsAdapter->abyStationAddress, abyStationAddress, sizeof(pLinkParmsAdapter->abyStationAddress));
        }

        pLinkParmsAdapter->bMaster = EC_FALSE;

        dwCcsrbar = sysGetPeripheralBase();

        pLinkParmsAdapter->dwPhyMdioBase = dwCcsrbar + 0x24000; /* eTSEC 0 */
        pLinkParmsAdapter->dwTbiPhyAddr = 16; /* Dummy address assigned to internal TBI PHY */
        pLinkParmsAdapter->oMiiBusMtx = EC_NULL; /* LinkOsCreateLock(eLockType_DEFAULT); */

        if (pLinkParmsAdapter->linkParms.dwInstance == 1) /* eTSEC1 (Assigned to VxWorks, don't use!) */
        {
           pLinkParmsAdapter->abyStationAddress[5] = 0x24;
           pLinkParmsAdapter->dwRegisterBase = dwCcsrbar + 0x24000;
           pLinkParmsAdapter->dwPhyAddr = 25; /* from hwconf.c */
#if (defined EC_VERSION_VXWORKS)
           pLinkParmsAdapter->dwRxInterrupt = 26; /* EPIC_TSEC1RX_INT_VEC from vxbEpicIntCtlr.h */
#endif
        }
        else if (pLinkParmsAdapter->linkParms.dwInstance == 2) /* eTSEC2 (This one is used by EtherCAT) */
        {
           pLinkParmsAdapter->abyStationAddress[5] = 0x25;
           pLinkParmsAdapter->dwRegisterBase = dwCcsrbar + 0x25000;
           pLinkParmsAdapter->dwPhyAddr = 26; /* from hwconf.c */
#if (defined EC_VERSION_VXWORKS)
           pLinkParmsAdapter->dwRxInterrupt = 32; /* EPIC_TSEC2RX_INT_VEC from vxbEpicIntCtlr.h */
#endif
        }

        pLinkParmsAdapter->dwLocalMdioBase = pLinkParmsAdapter->dwRegisterBase;
        break;
#else
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Invalid RefBoard value\n"));
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit; /* no break */
#endif

    /* Instron "XJ Electric Corp EPU20C" board */
    case eETSEC_XJ_EPU20C:
#if (defined EC_VERSION_VXWORKS)
        if (pLinkParmsAdapter->linkParms.dwInstance > 2) /* MPC8536 has 2 eTSEC's, but only first 2 are routed out to RJ45 ports */
        {
           EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Device unit must be 1 or 2!\n"));
           dwRetVal = EC_E_INVALIDPARM;
           goto Exit;
        }

        {
           EC_T_BYTE abyStationAddress[] = { 0x00, 0x02, 0xCE, 0x90, 0x02, 0x24 };
           memcpy(pLinkParmsAdapter->abyStationAddress, abyStationAddress, sizeof(pLinkParmsAdapter->abyStationAddress));
        }

        pLinkParmsAdapter->bMaster = EC_FALSE;
        dwCcsrbar = sysGetPeripheralBase();

        pLinkParmsAdapter->dwPhyMdioBase = dwCcsrbar + 0x24000; /* eTSEC 0 */
        pLinkParmsAdapter->dwTbiPhyAddr = 16; /* Dummy address assigned to internal TBI PHY */
        pLinkParmsAdapter->oMiiBusMtx = EC_NULL; /* LinkOsCreateLock(eLockType_DEFAULT); */

        if (pLinkParmsAdapter->linkParms.dwInstance == 1) /* eTSEC1 (Assigned to VxWorks, don't use!) */
        {
           pLinkParmsAdapter->abyStationAddress[5] = 0x24;
           pLinkParmsAdapter->dwRegisterBase = dwCcsrbar + 0x24000;
           pLinkParmsAdapter->dwPhyAddr = 0; /* from hwconf.c */
#if (defined EC_VERSION_VXWORKS)
           pLinkParmsAdapter->dwRxInterrupt = 26; /* EPIC_TSEC1RX_INT_VEC from vxbEpicIntCtlr.h */
#endif
        }
        else if (pLinkParmsAdapter->linkParms.dwInstance == 2) /* eTSEC2 (This one is used by EtherCAT) */
        {
           pLinkParmsAdapter->abyStationAddress[5] = 0x25;
           pLinkParmsAdapter->dwRegisterBase = dwCcsrbar + 0x26000;
           pLinkParmsAdapter->dwPhyAddr = 1; /* from hwconf.c */
#if (defined EC_VERSION_VXWORKS)
           pLinkParmsAdapter->dwRxInterrupt = 32; /* EPIC_TSEC2RX_INT_VEC from vxbEpicIntCtlr.h */
#endif
        }

        pLinkParmsAdapter->dwLocalMdioBase = pLinkParmsAdapter->dwRegisterBase;
        break;
#else
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Invalid RefBoard value\n"));
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit; /* no break */
#endif

    /* Freescale TWR-LS1021A-PB board (CPU LS1021A, Freescale-Linuxkernel 3.12)
        * Hardware resource informations (phy-addr, interrupts, io-base, ...)
        * are taken from Linux-kernel's device tree for TWR-LS1021A (ls1021a.dts and ls1021a.dtsi) */
    case eETSEC_TWRLS1021A:
        /* TWR-LS1021A-PB has 3 eTSEC's, but only two of them can be routed out to RJ45 ports */
        if ((2 < pLinkParmsAdapter->linkParms.dwInstance) || (1 > pLinkParmsAdapter->linkParms.dwInstance))
        {
           EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Device unit must be 1 or 2! 3. PHY on TWR-LS1012A-PB is not yet supported\n"));
           dwRetVal = EC_E_INVALIDPARM;
           goto Exit;
        }

        {
           EC_T_BYTE abyStationAddress[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xBA, 0xBE };
           memcpy(pLinkParmsAdapter->abyStationAddress, abyStationAddress, sizeof(pLinkParmsAdapter->abyStationAddress));
        }

        dwCcsrbar = 0x2D00000;

        /* Initialization and settings for ETSEC */
        pLinkParmsAdapter->bMaster = EC_TRUE;

        pLinkParmsAdapter->dwPhyMdioBase = dwCcsrbar + 0x24000; /* eTSEC 1, MDIO */
        pLinkParmsAdapter->dwTbiPhyAddr = 16; /* Dummy address assigned to internal TBI PHY */
        pLinkParmsAdapter->oMiiBusMtx = EC_NULL; /* LinkOsCreateLock(eLockType_DEFAULT); */

        /* Get interrupt number from Nth eTSEC device in PowerPC device tree */
        pLinkParmsAdapter->dwRxInterrupt = pLinkParmsAdapter->linkParms.dwInstance - 1;

#if (defined __arm__)
        pLinkParmsAdapter->bNotUseDmaBuffers = EC_TRUE;
#endif

        if (pLinkParmsAdapter->linkParms.dwInstance == 1) /* eTSEC1 */
        {
            pLinkParmsAdapter->dwRegisterBase = dwCcsrbar + 0x10000; /* eTSEC1, Group 0 */
            pLinkParmsAdapter->dwLocalMdioBase = dwCcsrbar + 0x24000; /* eTSEC1, MDIO */
#if (defined EC_VERSION_ETKERNEL)
            pLinkParmsAdapter->dwTbiPhyAddr = 0x1f;
#endif
            pLinkParmsAdapter->dwPhyAddr = 2;
        }
        else if (pLinkParmsAdapter->linkParms.dwInstance == 2) /* eTSEC2 */
        {
            pLinkParmsAdapter->dwRegisterBase = dwCcsrbar + 0x50000; /* eTSEC3, Group 0 */
            pLinkParmsAdapter->dwLocalMdioBase = dwCcsrbar + 0x24000; /* eTSEC3, MDIO */
#if (defined EC_VERSION_ETKERNEL)
            pLinkParmsAdapter->dwTbiPhyAddr = 0x1e;
#endif
            pLinkParmsAdapter->dwPhyAddr = 0;
        }
        else if (pLinkParmsAdapter->linkParms.dwInstance == 3) /* eTSEC3 */
        {
            pLinkParmsAdapter->dwRegisterBase = dwCcsrbar + 0x90000; /* eTSEC3, Group 0 */
            pLinkParmsAdapter->dwLocalMdioBase = dwCcsrbar + 0x24000; /* eTSEC3, MDIO */
            pLinkParmsAdapter->dwPhyAddr = 1;
        }
        break;

    /* TQMLS-LS102xA module (CPU LS1021) */
    case eETSEC_TQMLS_LS102XA:
        if (pLinkParmsAdapter->linkParms.dwInstance > 3)
        {
           EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Device unit must be <= 3!\n"));
           dwRetVal = EC_E_INVALIDPARM;
           goto Exit;
        }

        {
           EC_T_BYTE abyStationAddress[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xBA, 0xBE };
           memcpy(pLinkParmsAdapter->abyStationAddress, abyStationAddress, sizeof(pLinkParmsAdapter->abyStationAddress));
        }

        pLinkParmsAdapter->bMaster = EC_FALSE;

        dwCcsrbar = 0x2D00000;

        pLinkParmsAdapter->dwPhyMdioBase = dwCcsrbar + 0x24000; /* eTSEC 1, MDIO */
        pLinkParmsAdapter->dwTbiPhyAddr = 16; /* Dummy address assigned to internal TBI PHY */
        pLinkParmsAdapter->oMiiBusMtx = EC_NULL; /* LinkOsCreateLock(eLockType_DEFAULT); */

        /* Get interrupt number from Nth eTSEC device in PowerPC device tree */
        pLinkParmsAdapter->dwRxInterrupt = pLinkParmsAdapter->linkParms.dwInstance - 1;

#if (defined __arm__)
        pLinkParmsAdapter->bNotUseDmaBuffers = EC_TRUE;
#endif

        if (pLinkParmsAdapter->linkParms.dwInstance == 1) /* eTSEC1 */
        {
            pLinkParmsAdapter->dwRegisterBase = dwCcsrbar + 0x10000; /* eTSEC1, Group 0 */
            pLinkParmsAdapter->dwLocalMdioBase = dwCcsrbar + 0x24000; /* eTSEC1, MDIO */
            pLinkParmsAdapter->dwPhyAddr = 12;
        }
        else if (pLinkParmsAdapter->linkParms.dwInstance == 2) /* eTSEC2 */
        {
            pLinkParmsAdapter->dwRegisterBase = dwCcsrbar + 0x50000; /* eTSEC3, Group 0 */
            pLinkParmsAdapter->dwLocalMdioBase = dwCcsrbar + 0x24000; /* eTSEC3, MDIO */
            pLinkParmsAdapter->dwPhyAddr = 3;
        }
        else if (pLinkParmsAdapter->linkParms.dwInstance == 3) /* eTSEC3 */
        {
            pLinkParmsAdapter->dwRegisterBase = dwCcsrbar + 0x90000; /* eTSEC3, Group 0 */
            pLinkParmsAdapter->dwLocalMdioBase = dwCcsrbar + 0x24000; /* eTSEC3, MDIO */
            pLinkParmsAdapter->dwPhyAddr = 4;
        }

        break;
    }

    if (bCustom)
    {
        pLinkParmsAdapter->dwPhyAddr = dwCustomPhyAddr;
        pLinkParmsAdapter->dwRxInterrupt = dwCustomRxIrq;
        pLinkParmsAdapter->bNotUseDmaBuffers = bCustomNotUseDmaBuffers;
    }

    /* no errors */
    *ppLinkParms = &pLinkParmsAdapter->linkParms;
    dwRetVal = EC_E_NOERROR;

Exit:
    if (EC_E_NOERROR != dwRetVal)
    {
        SafeOsFree(pLinkParmsAdapter);
    }
    return dwRetVal;
}
#endif /* INCLUDE_EMLLETSEC */

#if (defined INCLUDE_EMLLFSLFEC)
/***************************************************************************************************/
/**
\brief  Create FslFec link layer parameters according to current command line parsing

\return EC_E_NOERROR     if link layer parameters was created
        EC_E_NOTFOUND    if command line was not matching
        EC_E_INVALIDPARM if syntax error
*/
static EC_T_DWORD CreateLinkParmsFromCmdLineFslFec(EC_T_CHAR** ptcWord, EC_T_CHAR** lpCmdLine, EC_T_CHAR* tcStorage, EC_T_BOOL* pbGetNextWord,
                                                   EC_T_LINK_PARMS** ppLinkParms)
{
EC_T_DWORD dwRetVal = EC_E_ERROR;
EC_T_LINK_PARMS_FSLFEC* pLinkParmsAdapter = EC_NULL;

    EC_UNREFPARM(pbGetNextWord);

    /* check for matching adapter */
    if (OsStricmp(*ptcWord, "-fslfec") != 0)
    {
        dwRetVal = EC_E_NOTFOUND;
        goto Exit;
    }
    /* alloc adapter specific link parms */
    pLinkParmsAdapter = (EC_T_LINK_PARMS_FSLFEC*)OsMalloc(sizeof(EC_T_LINK_PARMS_FSLFEC));
    if (EC_NULL == pLinkParmsAdapter)
    {
        dwRetVal = EC_E_NOMEMORY;
        goto Exit;
    }
    OsMemset(pLinkParmsAdapter, 0, sizeof(EC_T_LINK_PARMS_FSLFEC));
    LinkParmsInit(&pLinkParmsAdapter->linkParms, EC_LINK_PARMS_SIGNATURE_FSLFEC, sizeof(EC_T_LINK_PARMS_FSLFEC), EC_LINK_PARMS_IDENT_FSLFEC, 1, EcLinkMode_POLLING);

    /* parse mandatory parameters: instance, mode */
    if ( !ParseDword(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.dwInstance)
        || !ParseLinkMode(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.eLinkMode))
    {
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }
    pLinkParmsAdapter->dwRxBuffers = 96;
    pLinkParmsAdapter->dwTxBuffers = 96;

    /* get reference board */
    *ptcWord = GetNextWord(lpCmdLine, tcStorage);
    if      (0 == OsStricmp(*ptcWord, "mars"))     { pLinkParmsAdapter->eFecType = eFEC_IMX6;  pLinkParmsAdapter->ePhyInterface = ePHY_RGMII;      }
    else if (0 == OsStricmp(*ptcWord, "sabrelite")){ pLinkParmsAdapter->eFecType = eFEC_IMX6;  pLinkParmsAdapter->ePhyInterface = ePHY_RGMII;      }
    else if (0 == OsStricmp(*ptcWord, "sabresd"))  { pLinkParmsAdapter->eFecType = eFEC_IMX6;  pLinkParmsAdapter->ePhyInterface = ePHY_RGMII;      }
    else if (0 == OsStricmp(*ptcWord, "imx28evk"))
            if (pLinkParmsAdapter->linkParms.dwInstance == 1)
                                                     { pLinkParmsAdapter->eFecType = eFEC_IMX28; pLinkParmsAdapter->ePhyInterface = ePHY_RMII;
                                                       pLinkParmsAdapter->dwPhyAddr = 0;}
            else                                     { pLinkParmsAdapter->eFecType = eFEC_IMX28; pLinkParmsAdapter->ePhyInterface = ePHY_RMII;
                                                       pLinkParmsAdapter->dwPhyAddr = 1;}
    else if (0 == OsStricmp(*ptcWord, "topaz"))    { pLinkParmsAdapter->eFecType = eFEC_IMX25; pLinkParmsAdapter->ePhyInterface = ePHY_RMII;       }
    else if (0 == OsStricmp(*ptcWord, "imxceetul2"))
    {
        pLinkParmsAdapter->eFecType = eFEC_IMX6;
        pLinkParmsAdapter->ePhyInterface = ePHY_RMII;
        if (pLinkParmsAdapter->linkParms.dwInstance == 1)
        {
            pLinkParmsAdapter->dwPhyAddr = 1;
        }
        else if (pLinkParmsAdapter->linkParms.dwInstance == 2)
        {
            pLinkParmsAdapter->dwPhyAddr = 2;
        }
        else
        {
            dwRetVal = EC_E_INVALIDPARM;
            goto Exit;
        }
    }
    else if (0 == OsStricmp(*ptcWord, "mimxrt1064-evk"))
    {
        pLinkParmsAdapter->eFecType = eFEC_IMXRT1064;
        pLinkParmsAdapter->ePhyInterface = ePHY_RMII_50MHZ;
        if (pLinkParmsAdapter->linkParms.dwInstance == 1)
        {
            pLinkParmsAdapter->dwPhyAddr = 2;
        }
        else
        {
            dwRetVal = EC_E_INVALIDPARM;
            goto Exit;
        }
        pLinkParmsAdapter->dwRxBuffers = 8;
        pLinkParmsAdapter->dwTxBuffers = 8;
    }
    else if (0 == OsStricmp(*ptcWord, "custom"))
    {
        /* get FEC type */
        *ptcWord = GetNextWord(lpCmdLine, tcStorage);
        if      (0 == OsStricmp(*ptcWord, "imx25"))
        {
            pLinkParmsAdapter->eFecType         = eFEC_IMX25;
            pLinkParmsAdapter->dwRxInterrupt    = 57;
        }
        else if (0 == OsStricmp(*ptcWord, "imx28"))
        {
            pLinkParmsAdapter->eFecType         = eFEC_IMX28;
            pLinkParmsAdapter->dwRxInterrupt    = 57;
        }
        else if (0 == OsStricmp(*ptcWord, "imx53")) pLinkParmsAdapter->eFecType = eFEC_IMX53;
        else if (0 == OsStricmp(*ptcWord, "imx6"))  pLinkParmsAdapter->eFecType = eFEC_IMX6;
        else if (0 == OsStricmp(*ptcWord, "imx7"))  pLinkParmsAdapter->eFecType = eFEC_IMX7;
        else if (0 == OsStricmp(*ptcWord, "imx8m"))
        {
            pLinkParmsAdapter->eFecType         = eFEC_IMX8M;
            pLinkParmsAdapter->dwRxInterrupt    = 2;
        }
        else if (0 == OsStricmp(*ptcWord, "imx8"))
        {
            pLinkParmsAdapter->eFecType         = eFEC_IMX8;
            pLinkParmsAdapter->dwRxInterrupt    = 2;
        }
        else if (0 == OsStricmp(*ptcWord, "vf6"))
        {
            pLinkParmsAdapter->eFecType         = eFEC_VF6;
            pLinkParmsAdapter->dwRxInterrupt    = 78;
        }
        else if (0 == OsStricmp(*ptcWord, "imxrt1064"))
            {
                pLinkParmsAdapter->dwRxBuffers = 8;
                pLinkParmsAdapter->dwTxBuffers = 8;
                pLinkParmsAdapter->eFecType = eFEC_IMXRT1064;
            }
        else
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Invalid FecType value\n"));
            dwRetVal = EC_E_INVALIDPARM;
            goto Exit;
        }
        /* get PHY interface */
        *ptcWord = GetNextWord(lpCmdLine, tcStorage);
        if      (0 == OsStricmp(*ptcWord, "fixed")) pLinkParmsAdapter->ePhyInterface = ePHY_FIXED_LINK;
        else if (0 == OsStricmp(*ptcWord, "mii"))   pLinkParmsAdapter->ePhyInterface = ePHY_MII;
        else if (0 == OsStricmp(*ptcWord, "rmii"))  pLinkParmsAdapter->ePhyInterface = ePHY_RMII;
        else if (0 == OsStricmp(*ptcWord, "gmii"))  pLinkParmsAdapter->ePhyInterface = ePHY_GMII;
        else if (0 == OsStricmp(*ptcWord, "sgmii")) pLinkParmsAdapter->ePhyInterface = ePHY_SGMII;
        else if (0 == OsStricmp(*ptcWord, "rgmii")) pLinkParmsAdapter->ePhyInterface = ePHY_RGMII;
        else if (0 == OsStricmp(*ptcWord, "rmii50Mhz")) pLinkParmsAdapter->ePhyInterface = ePHY_RMII_50MHZ;
        else if (0 == OsStricmp((*ptcWord), "osdriver")) pLinkParmsAdapter->ePhyInterface = ePHY_OSDRIVER;
        else
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Invalid PhyInterface value\n"));
            dwRetVal = EC_E_INVALIDPARM;
            goto Exit;
        }
        /* get PHY address */
        if ((pLinkParmsAdapter->ePhyInterface != ePHY_OSDRIVER) &&
            !ParseDword(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->dwPhyAddr))
        {
            pLinkParmsAdapter->dwPhyAddr = 0;
        }
        (*ptcWord) = GetNextWord(lpCmdLine, tcStorage);
        if (((*ptcWord) != EC_NULL) && (0 == OsStricmp((*ptcWord), "nopinmuxing")))
        {
            pLinkParmsAdapter->bNoPinMuxing = EC_TRUE;
            (*ptcWord) = GetNextWord(lpCmdLine, tcStorage);
        }
        if (((*ptcWord) != EC_NULL) && (0 == OsStricmp((*ptcWord), "nomacaddr")))
        {
            pLinkParmsAdapter->bDontReadMacAddr = EC_TRUE;
        }
        else
        {
            *pbGetNextWord = EC_FALSE;
        }
    }
    else
    {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Invalid RefBoard value\n"));
            dwRetVal = EC_E_INVALIDPARM;
            goto Exit;
    }
    /* no errors */
    *ppLinkParms = &pLinkParmsAdapter->linkParms;
    dwRetVal = EC_E_NOERROR;

Exit:
    if (EC_E_NOERROR != dwRetVal)
    {
        SafeOsFree(pLinkParmsAdapter);
    }
    return dwRetVal;
}
#endif /* INCLUDE_EMLLFSLFEC */

#if (defined INCLUDE_EMLLGEM)

EC_T_BOOL ParseGEMType(
    EC_T_CHAR**     ptcWord,
    EC_T_CHAR**     lpCmdLine,
    EC_T_CHAR*      tcStorage,
    EC_T_GEM_TYPE*  peType)
{
    EC_T_BOOL bRes = EC_FALSE;

    if (ParseString(ptcWord, lpCmdLine, tcStorage))
    {
        bRes = EC_TRUE;
        if (0 == OsStricmp(*ptcWord, "zynq7000"))         { *peType = eGemType_Zynq7000; }
        else if (0 == OsStricmp(*ptcWord, "ultrascale"))  { *peType = eGemType_ZynqUltrascale; }
        else
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Invalid GEM type %s\n", ptcWord));
            bRes = EC_FALSE;
        }
    }
    return bRes;
}

/***************************************************************************************************/
/**
\brief  Create GEM link layer parameters according to current command line parsing

\return EC_E_NOERROR     if link layer parameters was created
        EC_E_NOTFOUND    if command line was not matching
        EC_E_INVALIDPARM if syntax error
*/
static EC_T_DWORD CreateLinkParmsFromCmdLineGEM(EC_T_CHAR** ptcWord, EC_T_CHAR** lpCmdLine, EC_T_CHAR* tcStorage, EC_T_BOOL* pbGetNextWord,
                                                EC_T_LINK_PARMS** ppLinkParms)
{
EC_T_DWORD dwRetVal = EC_E_ERROR;
EC_T_LINK_PARMS_GEM* pLinkParmsAdapter = EC_NULL;
EC_T_BOOL bGetNextWord = EC_TRUE;

    /* check for matching adapter */
    if (OsStricmp(*ptcWord, "-gem") != 0)
    {
        dwRetVal = EC_E_NOTFOUND;
        goto Exit;
    }
    /* alloc adapter specific link parms */
    pLinkParmsAdapter = (EC_T_LINK_PARMS_GEM*)OsMalloc(sizeof(EC_T_LINK_PARMS_GEM));
    if (EC_NULL == pLinkParmsAdapter)
    {
        dwRetVal = EC_E_NOMEMORY;
        goto Exit;
    }
    OsMemset(pLinkParmsAdapter, 0, sizeof(EC_T_LINK_PARMS_GEM));
    LinkParmsInit(&pLinkParmsAdapter->linkParms, EC_LINK_PARMS_SIGNATURE_GEM, sizeof(EC_T_LINK_PARMS_GEM), EC_LINK_PARMS_IDENT_GEM, 1, EcLinkMode_POLLING);

    /* parse mandatory parameters */
    if ( !ParseDword(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.dwInstance)
      || !ParseLinkMode(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.eLinkMode))
    {
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }

    if (pLinkParmsAdapter->linkParms.dwInstance < 1 || pLinkParmsAdapter->linkParms.dwInstance > 4)
    {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Instance number must be  within 1 to 4\n"));
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }

    /* default parameters */
    pLinkParmsAdapter->dwRxInterrupt = pLinkParmsAdapter->linkParms.dwInstance - 1;
    pLinkParmsAdapter->ePhyInterface = ePHY_RGMII;
    pLinkParmsAdapter->bUseGmiiToRgmiiConv = EC_FALSE;
    pLinkParmsAdapter->dwConvPhyAddr = 0;
    pLinkParmsAdapter->bUseDmaBuffers = EC_FALSE;
    pLinkParmsAdapter->bNoPhyAccess = EC_FALSE;   /* Link layer should initialize PHY and read link status (connected/disconnected) */

    /* get reference board */
    *ptcWord = GetNextWord(lpCmdLine, tcStorage);
    if ((EC_NULL != *ptcWord) && (0 == OsStricmp(*ptcWord, "microzed")))
    {
        pLinkParmsAdapter->eGemType = eGemType_Zynq7000;
        pLinkParmsAdapter->dwPhyAddr = 4;
        pLinkParmsAdapter->eRxSource = eGemRxSource_MIO;
    }
    else if ((EC_NULL != *ptcWord) && (0 == OsStricmp(*ptcWord, "zedboard")))
    {
        pLinkParmsAdapter->eGemType = eGemType_Zynq7000;
        pLinkParmsAdapter->dwPhyAddr = 0;
        pLinkParmsAdapter->eRxSource = eGemRxSource_MIO;
    }
    else if ((EC_NULL != *ptcWord) && (0 == OsStricmp(*ptcWord, "zc702")))
    {
        pLinkParmsAdapter->eGemType = eGemType_Zynq7000;
        pLinkParmsAdapter->dwPhyAddr = 7;
        pLinkParmsAdapter->eRxSource = eGemRxSource_MIO;
    }
    else if ((EC_NULL != *ptcWord) && ((0 == OsStricmp(*ptcWord, "zcu102")) || (0 == OsStricmp(*ptcWord, "zcu104"))))
    {
        pLinkParmsAdapter->eGemType         = eGemType_ZynqUltrascale;
        pLinkParmsAdapter->dwPhyAddr        = 12;
        pLinkParmsAdapter->eRxSource        = eGemRxSource_MIO;
        pLinkParmsAdapter->dwRxInterrupt    = 1;
        pLinkParmsAdapter->eClkDivType      = eGemClkDivType_default;
    }
    else if ((EC_NULL != *ptcWord) && ((0 == OsStricmp(*ptcWord, "KR260"))))
    {
        pLinkParmsAdapter->eGemType         = eGemType_ZynqUltrascale;
        pLinkParmsAdapter->eRxSource        = eGemRxSource_MIO;
        pLinkParmsAdapter->dwRxInterrupt    = 1;
        pLinkParmsAdapter->eClkDivType      = eGemClkDivType_K26;
        pLinkParmsAdapter->bNoPinMuxing     = EC_TRUE;

        if (2 == pLinkParmsAdapter->linkParms.dwInstance)
        {
        	pLinkParmsAdapter->ePhyInterface    = ePHY_RGMII;
            pLinkParmsAdapter->dwPhyAddr        = 8;
        }
        /* instance 1 is not supported */
    }
    else if ((EC_NULL != *ptcWord) && (0 == OsStricmp(*ptcWord, "custom")))
    {
        EC_T_DWORD dwUseGmiiToRgmiiConv = 0;

        if (!ParseDword(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->dwPhyAddr)
         || !ParseDword(ptcWord, lpCmdLine, tcStorage, (EC_T_DWORD*)&pLinkParmsAdapter->eRxSource)
         || !ParseDword(ptcWord, lpCmdLine, tcStorage, &dwUseGmiiToRgmiiConv)
         || !ParseDword(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->dwConvPhyAddr)
         || !ParseGEMType(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->eGemType))
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Parse custom parameters failed!\n"));
            dwRetVal = EC_E_INVALIDPARM;
            goto Exit;
        }
        pLinkParmsAdapter->bUseGmiiToRgmiiConv = (0 != dwUseGmiiToRgmiiConv);
    }
    else
    {
        /* default: IXXAT Econ 100*/
        pLinkParmsAdapter->eGemType = eGemType_Zynq7000;
        if (1 == pLinkParmsAdapter->linkParms.dwInstance)
        {
            pLinkParmsAdapter->dwPhyAddr = 4;
            pLinkParmsAdapter->eRxSource = eGemRxSource_MIO;
        }
        else if (2 == pLinkParmsAdapter->linkParms.dwInstance)
        {
            pLinkParmsAdapter->dwPhyAddr = 1;
            pLinkParmsAdapter->eRxSource = eGemRxSource_EMIO;
        }
        else
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Instance number for Z7000 must be 1 or 2\n"));
            dwRetVal = EC_E_INVALIDPARM;
            goto Exit;
        }
        bGetNextWord = EC_FALSE;
    }
    /* osDriver */
    if (bGetNextWord)
    {
        *ptcWord = GetNextWord(lpCmdLine, tcStorage);
    }
    if ((EC_NULL != *ptcWord) && (0 == OsStricmp((*ptcWord), "osdriver")))
    {
        pLinkParmsAdapter->ePhyInterface = ePHY_OSDRIVER;
        *ptcWord = GetNextWord(lpCmdLine, tcStorage);
    }
    if ((EC_NULL != *ptcWord) && (0 == OsStricmp((*ptcWord), "ClkDivType_K26")))
    {
        pLinkParmsAdapter->eClkDivType = eGemClkDivType_K26;
        *ptcWord = GetNextWord(lpCmdLine, tcStorage);
    }
    if ((EC_NULL != *ptcWord) && (0 == OsStricmp((*ptcWord), "nopinmuxing")))
    {
        pLinkParmsAdapter->bNoPinMuxing = EC_TRUE;
        bGetNextWord = EC_TRUE;
    }
    else
    {
        bGetNextWord = EC_FALSE;
    }
    if (EC_NULL != pbGetNextWord)
    {
        *pbGetNextWord = bGetNextWord;
    }
    /* no errors */
    *ppLinkParms = &pLinkParmsAdapter->linkParms;
    dwRetVal = EC_E_NOERROR;

Exit:
    if (EC_E_NOERROR != dwRetVal)
    {
        SafeOsFree(pLinkParmsAdapter);
    }
    return dwRetVal;
}
#endif /* INCLUDE_EMLLGEM */

#if (defined INCLUDE_EMLLI8254X)
/***************************************************************************************************/
/**
\brief  Create I8254x link layer parameters according to current command line parsing

\return EC_E_NOERROR     if link layer parameters was created
        EC_E_NOTFOUND    if command line was not matching
        EC_E_INVALIDPARM if syntax error
*/
static EC_T_DWORD CreateLinkParmsFromCmdLineI8254x(EC_T_CHAR** ptcWord, EC_T_CHAR** lpCmdLine, EC_T_CHAR* tcStorage, EC_T_BOOL* pbGetNextWord,
                                                   EC_T_LINK_PARMS** ppLinkParms)
{
EC_T_DWORD dwRetVal = EC_E_ERROR;
EC_T_LINK_PARMS_I8254X* pLinkParmsAdapter = EC_NULL;

    EC_UNREFPARM(pbGetNextWord);

    /* check for matching adapter */
    if (OsStricmp(*ptcWord, "-i8254x") != 0)
    {
        dwRetVal = EC_E_NOTFOUND;
        goto Exit;
    }
    /* alloc adapter specific link parms */
    pLinkParmsAdapter = (EC_T_LINK_PARMS_I8254X*)OsMalloc(sizeof(EC_T_LINK_PARMS_I8254X));
    if (EC_NULL == pLinkParmsAdapter)
    {
        dwRetVal = EC_E_NOMEMORY;
        goto Exit;
    }
    OsMemset(pLinkParmsAdapter, 0, sizeof(EC_T_LINK_PARMS_I8254X));
    LinkParmsInit(&pLinkParmsAdapter->linkParms, EC_LINK_PARMS_SIGNATURE_I8254X, sizeof(EC_T_LINK_PARMS_I8254X), EC_LINK_PARMS_IDENT_I8254X, 1, EcLinkMode_POLLING);

    /* parse mandatory parameters: instance, mode */
    if ( !ParseDword(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.dwInstance)
        || !ParseLinkMode(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.eLinkMode))
    {
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }

#if ((EC_ARCH == EC_ARCH_ARM) || (EC_ARCH == EC_ARCH_ARM64))
    pLinkParmsAdapter->wRxBufferCnt = 32;
    pLinkParmsAdapter->wTxBufferCnt = 32;
    pLinkParmsAdapter->bNotUseDmaBuffers = EC_TRUE;
#endif

    /* no errors */
    *ppLinkParms = &pLinkParmsAdapter->linkParms;
    dwRetVal = EC_E_NOERROR;

Exit:
    if (EC_E_NOERROR != dwRetVal)
    {
        SafeOsFree(pLinkParmsAdapter);
    }
    return dwRetVal;
}
#endif /* INCLUDE_EMLLI8254X */

#if (defined INCLUDE_EMLLI8255X)
/***************************************************************************************************/
/**
\brief  Create I8255X link layer parameters according to current command line parsing

\return EC_E_NOERROR     if link layer parameters was created
        EC_E_NOTFOUND    if command line was not matching
        EC_E_INVALIDPARM if syntax error
*/
static EC_T_DWORD CreateLinkParmsFromCmdLineI8255x(EC_T_CHAR** ptcWord, EC_T_CHAR** lpCmdLine, EC_T_CHAR* tcStorage, EC_T_BOOL* pbGetNextWord,
                                                   EC_T_LINK_PARMS** ppLinkParms)
{
EC_T_DWORD dwRetVal = EC_E_ERROR;
EC_T_LINK_PARMS_I8255X* pLinkParmsAdapter = EC_NULL;

    EC_UNREFPARM(pbGetNextWord);

    /* check for matching adapter */
    if (OsStricmp(*ptcWord, "-i8255x") != 0)
    {
        dwRetVal = EC_E_NOTFOUND;
        goto Exit;
    }
    /* alloc adapter specific link parms */
    pLinkParmsAdapter = (EC_T_LINK_PARMS_I8255X*)OsMalloc(sizeof(EC_T_LINK_PARMS_I8255X));
    if (EC_NULL == pLinkParmsAdapter)
    {
        dwRetVal = EC_E_NOMEMORY;
        goto Exit;
    }
    OsMemset(pLinkParmsAdapter, 0, sizeof(EC_T_LINK_PARMS_I8255X));
    LinkParmsInit(&pLinkParmsAdapter->linkParms, EC_LINK_PARMS_SIGNATURE_I8255X, sizeof(EC_T_LINK_PARMS_I8255X), EC_LINK_PARMS_IDENT_I8255X, 1, EcLinkMode_POLLING);

    /* parse mandatory parameters: instance, mode */
    if ( !ParseDword(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.dwInstance)
        || !ParseLinkMode(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.eLinkMode))
    {
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }
    /* no errors */
    *ppLinkParms = &pLinkParmsAdapter->linkParms;
    dwRetVal = EC_E_NOERROR;

Exit:
    if (EC_E_NOERROR != dwRetVal)
    {
        SafeOsFree(pLinkParmsAdapter);
    }
    return dwRetVal;
}
#endif /* INCLUDE_EMLLI8255X */

#if (defined INCLUDE_EMLLICSS)
static EC_T_VOID cbTtsStartCycle(EC_T_VOID* pvContext)
{
    OsSetEvent(pvContext);
}
/***************************************************************************************************/
/**
\brief  Create ICSS link layer parameters according to current command line parsing

\return EC_E_NOERROR     if link layer parameters was created
EC_E_NOTFOUND    if command line was not matching
EC_E_INVALIDPARM if syntax error
*/
static EC_T_DWORD CreateLinkParmsFromCmdLineICSS(EC_T_CHAR** ptcWord, EC_T_CHAR** lpCmdLine, EC_T_CHAR* tcStorage, EC_T_BOOL* pbGetNextWord,
                                                 EC_T_LINK_PARMS** ppLinkParms, EC_T_LINK_TTS* pTtsParms)
{
    EC_T_DWORD dwRetVal = EC_E_ERROR;
    EC_T_LINK_PARMS_ICSS* pLinkParmsAdapter = EC_NULL;

    EC_UNREFPARM(pbGetNextWord);

    /* check for matching adapter */
    if (OsStricmp(*ptcWord, "-icss") != 0)
    {
        dwRetVal = EC_E_NOTFOUND;
        goto Exit;
    }
    /* alloc adapter specific link parms */
    pLinkParmsAdapter = (EC_T_LINK_PARMS_ICSS*)OsMalloc(sizeof(EC_T_LINK_PARMS_ICSS));
    if (EC_NULL == pLinkParmsAdapter)
    {
        dwRetVal = EC_E_NOMEMORY;
        goto Exit;
    }
    OsMemset(pLinkParmsAdapter, 0, sizeof(EC_T_LINK_PARMS_ICSS));
    LinkParmsInit(&pLinkParmsAdapter->linkParms, EC_LINK_PARMS_SIGNATURE_ICSS, sizeof(EC_T_LINK_PARMS_ICSS), EC_LINK_PARMS_IDENT_ICSS, 1, EcLinkMode_POLLING);

    if (!ParseDword(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.dwInstance)
     || !ParseLinkMode(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.eLinkMode))
    {
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }
    if ((pLinkParmsAdapter->linkParms.dwInstance < 1) || (pLinkParmsAdapter->linkParms.dwInstance > 4))
    {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ICSS: Port number must be between 1 and 4\n"));
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }

    /* get master/slave flag */
    *ptcWord = GetNextWord(lpCmdLine, tcStorage);
    if ((0 != *ptcWord) && (0 == OsStricmp(*ptcWord, "m")))
    {
        pLinkParmsAdapter->bMaster = EC_TRUE;
    }
    else if ((0 != *ptcWord) && (0 == OsStricmp(*ptcWord, "s")))
    {
        pLinkParmsAdapter->bMaster = EC_FALSE;
    }
    else
    {
        EcLogMsg(EC_LOG_LEVEL_WARNING, (pEcLogContext, EC_LOG_LEVEL_WARNING, "ICSS: No master/slave flag specified. Assume master\n"));
        pLinkParmsAdapter->bMaster = EC_TRUE;
        *pbGetNextWord = EC_FALSE;
    }

    /* get reference board */
    if (*pbGetNextWord)
    {
        *ptcWord = GetNextWord(lpCmdLine, tcStorage);
    }
    if ((0 != *ptcWord) && (0 == OsStricmp(*ptcWord, "am572x-idk")))
    {
        pLinkParmsAdapter->eBoardType = EcLinkIcssBoard_am572x;
    }
    else if ((0 != *ptcWord) && (0 == OsStricmp(*ptcWord, "am571x-idk")))
    {
        pLinkParmsAdapter->eBoardType = EcLinkIcssBoard_am571x;
    }
    else if ((0 != *ptcWord) && (0 == OsStricmp(*ptcWord, "am574x")))
    {
        pLinkParmsAdapter->eBoardType = EcLinkIcssBoard_am574x;
    }
    else if ((0 != *ptcWord) && (0 == OsStricmp(*ptcWord, "am3359-icev2")))
    {
        pLinkParmsAdapter->eBoardType = EcLinkIcssBoard_am3359;
    }
    else if ((0 != *ptcWord) && (0 == OsStricmp(*ptcWord, "am572x-emerson")))
    {
        pLinkParmsAdapter->eBoardType = EcLinkIcssBoard_am572x_emerson;
    }
    else
    {
        EcLogMsg(EC_LOG_LEVEL_WARNING, (pEcLogContext, EC_LOG_LEVEL_WARNING, "ICSS: No board name specified. Assume am572x-idk\n"));
        pLinkParmsAdapter->eBoardType = EcLinkIcssBoard_am572x;
        *pbGetNextWord = EC_FALSE;
    }

    if (*pbGetNextWord)
    {
        *ptcWord = GetNextWord(lpCmdLine, tcStorage);
    }

    /* PHY interfaces and PHY address*/
    if (((*ptcWord) != EC_NULL) && (0 == OsStricmp((*ptcWord), "mii")))
    {
        EC_T_DWORD dwPhyAddr = 0;

        if (!ParseDword(ptcWord, lpCmdLine, tcStorage, &dwPhyAddr))
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ICSS: PhyAddr missing \n"));
            dwRetVal = EC_E_INVALIDPARM;
            goto Exit;
        }

        (*ptcWord) = GetNextWord(lpCmdLine, tcStorage);
        pLinkParmsAdapter->ePhyInterface = ePHY_MII;

        pLinkParmsAdapter->dwPhyAddr = dwPhyAddr;
    }
    else
    {
        if (((*ptcWord) != EC_NULL) && (0 == OsStricmp((*ptcWord), "osdriver")))
        {
            pLinkParmsAdapter->ePhyInterface = ePHY_OSDRIVER;

            (*ptcWord) = GetNextWord(lpCmdLine, tcStorage);
        }
        else
        {
            pLinkParmsAdapter->ePhyInterface = ePHY_MII;
        }

        if (EcLinkIcssBoard_am3359 == pLinkParmsAdapter->eBoardType)
        {
            switch (pLinkParmsAdapter->linkParms.dwInstance)
            {
            case 1: pLinkParmsAdapter->dwPhyAddr = 1;   break;
            case 2: pLinkParmsAdapter->dwPhyAddr = 3;   break;
            default: pLinkParmsAdapter->dwPhyAddr = 0;
            }
        }
        else
        {
            switch (pLinkParmsAdapter->linkParms.dwInstance)
            {
            case 1: pLinkParmsAdapter->dwPhyAddr = 0;   break;
            case 2: pLinkParmsAdapter->dwPhyAddr = 1;   break;
            case 3: pLinkParmsAdapter->dwPhyAddr = 0;   break;
            case 4: pLinkParmsAdapter->dwPhyAddr = 1;   break;
            default: pLinkParmsAdapter->dwPhyAddr = 0;
            }
        }
    }

    /* disable PHY reset */
    if (((*ptcWord) != EC_NULL) && (0 == OsStricmp((*ptcWord), "noPhyReset")))
    {
        pLinkParmsAdapter->bNoPhyReset = EC_TRUE;
        (*ptcWord) = GetNextWord(lpCmdLine, tcStorage);
    }

    /* get TTS mode and config time */
    if (((*ptcWord) != EC_NULL) && (0 == OsStricmp(*ptcWord, "tts")))
    {
        if (!ParseDword(ptcWord, lpCmdLine, tcStorage, &pTtsParms->dwCycleTimeUsec) || pTtsParms->dwCycleTimeUsec < 125)
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ICSS: Wrong TTS cycle period specified\n"));
            dwRetVal = EC_E_INVALIDPARM;
            goto Exit;
        }

        if (!ParseDword(ptcWord, lpCmdLine, tcStorage, &pTtsParms->dwSendOffsetUsec))
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ICSS: Wrong TTS config time specified\n"));
            dwRetVal = EC_E_INVALIDPARM;
            goto Exit;
        }
        pTtsParms->bEnabled = EC_TRUE;
        pTtsParms->pfnStartCycle = cbTtsStartCycle;
        pTtsParms->pvStartCycleContext = OsCreateEvent();

        OsMemcpy(&pLinkParmsAdapter->TtsParms, pTtsParms, sizeof(EC_T_LINK_TTS));
    }
    else
    {
        *pbGetNextWord = EC_FALSE;
    }

    /* no errors */
    *ppLinkParms = &pLinkParmsAdapter->linkParms;
    dwRetVal = EC_E_NOERROR;

Exit:
    if (EC_E_NOERROR != dwRetVal)
    {
        SafeOsFree(pLinkParmsAdapter);
    }
    return dwRetVal;
}
#endif /* INCLUDE_EMLLICSS */

#if (defined INCLUDE_EMLLICSSG)

/***************************************************************************************************/
/**
\brief  Create ICSSG link layer parameters according to current command line parsing

\return EC_E_NOERROR     if link layer parameters was created
EC_E_NOTFOUND    if command line was not matching
EC_E_INVALIDPARM if syntax error
*/
static EC_T_DWORD CreateLinkParmsFromCmdLineICSSG(EC_T_CHAR** ptcWord, EC_T_CHAR** lpCmdLine, EC_T_CHAR* tcStorage, EC_T_BOOL* pbGetNextWord,
                                                 EC_T_LINK_PARMS** ppLinkParms, EC_T_LINK_TTS* pTtsParms)
{
    EC_T_DWORD dwRetVal = EC_E_ERROR;
    EC_T_LINK_PARMS_ICSSG* pLinkParmsAdapter = EC_NULL;
    EC_UNREFPARM(pbGetNextWord);

    /* check for matching adapter */
    if (OsStricmp(*ptcWord, "-icssg") != 0)
    {
        dwRetVal = EC_E_NOTFOUND;
        goto Exit;
    }
    /* alloc adapter specific link parms */
    pLinkParmsAdapter = (EC_T_LINK_PARMS_ICSSG*)OsMalloc(sizeof(EC_T_LINK_PARMS_ICSSG));
    if (EC_NULL == pLinkParmsAdapter)
    {
        dwRetVal = EC_E_NOMEMORY;
        goto Exit;
    }
    OsMemset(pLinkParmsAdapter, 0, sizeof(EC_T_LINK_PARMS_ICSSG));
    LinkParmsInit(&pLinkParmsAdapter->linkParms, EC_LINK_PARMS_SIGNATURE_ICSSG, sizeof(EC_T_LINK_PARMS_ICSSG), EC_LINK_PARMS_IDENT_ICSSG, 1, EcLinkMode_POLLING);

    if (!ParseDword(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.dwInstance)
     || !ParseLinkMode(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.eLinkMode))
    {
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }

    /* get master/slave flag */
    *ptcWord = GetNextWord(lpCmdLine, tcStorage);
    if ((0 != *ptcWord) && (0 == OsStricmp(*ptcWord, "m")))
    {
        pLinkParmsAdapter->bMaster = EC_TRUE;
    }
    else if ((0 != *ptcWord) && (0 == OsStricmp(*ptcWord, "s")))
    {
        pLinkParmsAdapter->bMaster = EC_FALSE;
    }
    else
    {
        EcLogMsg(EC_LOG_LEVEL_WARNING, (pEcLogContext, EC_LOG_LEVEL_WARNING, "ICSSG: No master/slave flag specified. Assume master\n"));
        pLinkParmsAdapter->bMaster = EC_TRUE;
        *pbGetNextWord = EC_FALSE;
    }

    /* get reference board */
    if (*pbGetNextWord)
    {
        *ptcWord = GetNextWord(lpCmdLine, tcStorage);
    }
    if ((0 != *ptcWord) && (0 == OsStricmp(*ptcWord, "am654x-idk")))
    {
        pLinkParmsAdapter->eBoardType = EcLinkIcssgBoard_am654x;
    }
    else
    {
        EcLogMsg(EC_LOG_LEVEL_WARNING, (pEcLogContext, EC_LOG_LEVEL_WARNING, "ICSSG: No board name specified. Assume am654x-idk\n"));
        pLinkParmsAdapter->eBoardType = EcLinkIcssgBoard_am654x;
        *pbGetNextWord = EC_FALSE;
    }

    /* no errors */
    *ppLinkParms = &pLinkParmsAdapter->linkParms;
    dwRetVal = EC_E_NOERROR;
Exit:
    if (EC_E_NOERROR != dwRetVal)
    {
        SafeOsFree(pLinkParmsAdapter);
    }
    return dwRetVal;
}

#endif /* INCLUDE_EMLLICSSG */

#if (defined INCLUDE_EMLLL9218I)
/***************************************************************************************************/
/**
\brief  Create L9218I link layer parameters according to current command line parsing

\return EC_E_NOERROR     if link layer parameters was created
        EC_E_NOTFOUND    if command line was not matching
        EC_E_INVALIDPARM if syntax error
*/
static EC_T_DWORD CreateLinkParmsFromCmdLineL9218i(EC_T_CHAR** ptcWord, EC_T_CHAR** lpCmdLine, EC_T_CHAR* tcStorage, EC_T_BOOL* pbGetNextWord,
                                                   EC_T_LINK_PARMS** ppLinkParms)
{
EC_T_DWORD dwRetVal = EC_E_ERROR;
EC_T_LINK_PARMS_L9218I* pLinkParmsAdapter = EC_NULL;

    EC_UNREFPARM(ptcWord);
    EC_UNREFPARM(lpCmdLine);
    EC_UNREFPARM(tcStorage);
    EC_UNREFPARM(pbGetNextWord);

    /* check for matching adapter */
    if (OsStricmp(*ptcWord, "-l9218i") != 0)
    {
        dwRetVal = EC_E_NOTFOUND;
        goto Exit;
    }
    /* alloc adapter specific link parms */
    pLinkParmsAdapter = (EC_T_LINK_PARMS_L9218I*)OsMalloc(sizeof(EC_T_LINK_PARMS_L9218I));
    if (EC_NULL == pLinkParmsAdapter)
    {
        dwRetVal = EC_E_NOMEMORY;
        goto Exit;
    }
    OsMemset(pLinkParmsAdapter, 0, sizeof(EC_T_LINK_PARMS_L9218I));
    LinkParmsInit(&pLinkParmsAdapter->linkParms, EC_LINK_PARMS_SIGNATURE_L9218I, sizeof(EC_T_LINK_PARMS_L9218I), EC_LINK_PARMS_IDENT_L9218I, 1, EcLinkMode_POLLING);

    /* get mode */
    if (!ParseLinkMode(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.eLinkMode))
    {
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }
    /* no errors */
    *ppLinkParms = &pLinkParmsAdapter->linkParms;
    dwRetVal = EC_E_NOERROR;

Exit:
    if (EC_E_NOERROR != dwRetVal)
    {
        SafeOsFree(pLinkParmsAdapter);
    }
    return dwRetVal;
}
#endif /* INCLUDE_EMLLL9218I */

#if (defined INCLUDE_EMLLLAN743X)
/***************************************************************************************************/
/**
\brief  Create LAN743x link layer parameters according to current command line parsing

\return EC_E_NOERROR     if link layer parameters was created
        EC_E_NOTFOUND    if command line was not matching
        EC_E_INVALIDPARM if syntax error
*/
static EC_T_DWORD CreateLinkParmsFromCmdLineLAN743x(EC_T_CHAR** ptcWord, EC_T_CHAR** lpCmdLine, EC_T_CHAR* tcStorage, EC_T_BOOL* pbGetNextWord,
    EC_T_LINK_PARMS** ppLinkParms)
{
    EC_T_DWORD dwRetVal = EC_E_ERROR;
    EC_T_LINK_PARMS_LAN743X* pLinkParmsAdapter = EC_NULL;

    EC_UNREFPARM(pbGetNextWord);

    /* check for matching adapter */
    if (OsStricmp(*ptcWord, "-lan743x") != 0)
    {
        dwRetVal = EC_E_NOTFOUND;
        goto Exit;
    }
    /* alloc adapter specific link parms */
    pLinkParmsAdapter = (EC_T_LINK_PARMS_LAN743X*)OsMalloc(sizeof(EC_T_LINK_PARMS_LAN743X));
    if (EC_NULL == pLinkParmsAdapter)
    {
        dwRetVal = EC_E_NOMEMORY;
        goto Exit;
    }
    OsMemset(pLinkParmsAdapter, 0, sizeof(EC_T_LINK_PARMS_LAN743X));
    LinkParmsInit(&pLinkParmsAdapter->linkParms, EC_LINK_PARMS_SIGNATURE_LAN743X, sizeof(EC_T_LINK_PARMS_LAN743X), EC_LINK_PARMS_IDENT_LAN743X, 1, EcLinkMode_POLLING);

    /* parse mandatory parameters: instance, mode */
    if (!ParseDword(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.dwInstance)
        || !ParseLinkMode(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.eLinkMode))
    {
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }
    /* no errors */
    *ppLinkParms = &pLinkParmsAdapter->linkParms;
    dwRetVal = EC_E_NOERROR;

Exit:
    if (EC_E_NOERROR != dwRetVal)
    {
        SafeOsFree(pLinkParmsAdapter);
    }
    return dwRetVal;
}
#endif /* INCLUDE_EMLLLAN743X */

#if (defined INCLUDE_EMLLNDISUIO)
/***************************************************************************************************/
/**
\brief  Create NDISUIO link layer parameters according to current command line parsing

\return EC_E_NOERROR     if link layer parameters was created
        EC_E_NOTFOUND    if command line was not matching
        EC_E_INVALIDPARM if syntax error
*/
static EC_T_DWORD CreateLinkParmsFromCmdLineNDISUIO(EC_T_CHAR** ptcWord, EC_T_CHAR** lpCmdLine, EC_T_CHAR* tcStorage, EC_T_BOOL* pbGetNextWord,
                                                    EC_T_LINK_PARMS** ppLinkParms)
{
EC_T_DWORD dwRetVal = EC_E_ERROR;
EC_T_LINK_PARMS_NDISUIO* pLinkParmsAdapter = EC_NULL;

#define NDISUIO_DEVNAME   TEXT("ECT1:")
#define NDISUIO_DRIVERKEY TEXT("Drivers\\BuiltIn\\ECAT")
HANDLE      hNdisUioDevice  = EC_NULL;
HANDLE      hNdisUioDriver  = EC_NULL;

    EC_UNREFPARM(pbGetNextWord);

    /* check for matching adapter */
    if (OsStricmp(*ptcWord, "-ndisuio") != 0)
    {
        dwRetVal = EC_E_NOTFOUND;
        goto Exit;
    }
    /* get next word */
    *ptcWord = GetNextWord(lpCmdLine, tcStorage);
    if ((*ptcWord == EC_NULL) || (OsStrncmp(*ptcWord, "-", 1) == 0))
    {
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }
    /* alloc adapter specific link parms */
    pLinkParmsAdapter = (EC_T_LINK_PARMS_NDISUIO*)OsMalloc(sizeof(EC_T_LINK_PARMS_NDISUIO));
    if (EC_NULL == pLinkParmsAdapter)
    {
        dwRetVal = EC_E_NOMEMORY;
        goto Exit;
    }
    OsMemset(pLinkParmsAdapter, 0, sizeof(EC_T_LINK_PARMS_NDISUIO));
    LinkParmsInit(&pLinkParmsAdapter->linkParms, EC_LINK_PARMS_SIGNATURE_NDISUIO, sizeof(EC_T_LINK_PARMS_NDISUIO), EC_LINK_PARMS_IDENT_NDISUIO, 1, EcLinkMode_POLLING);

    /* check if NDISUIO driver started */
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Check if ECAT driver is started..."));
    hNdisUioDevice = CreateFile( NDISUIO_DEVNAME, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
    if ((hNdisUioDevice != EC_NULL) && (hNdisUioDevice != INVALID_HANDLE_VALUE))
    {
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Ok!\n"));
    }
    else
    {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Failed!\n"));
        hNdisUioDevice = EC_NULL;
    }
    /* try to load driver if not already loaded */
    if (hNdisUioDevice == EC_NULL)
    {
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Try to load ECAT driver..."));
        hNdisUioDriver = ActivateDeviceEx (NDISUIO_DRIVERKEY, 0,  0, 0);
        if ((hNdisUioDriver != EC_NULL) && (hNdisUioDriver != INVALID_HANDLE_VALUE))
        {
            EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Ok!\n"));
        }
        else
        {
            EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Failed!\n"));
            hNdisUioDriver = EC_NULL;
            dwRetVal = EC_E_INVALIDSTATE;
            goto Exit;
        }
    }
    /* check if driver available */
    if ((hNdisUioDevice == EC_NULL) && (hNdisUioDriver == EC_NULL))
    {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "ERROR: No NDISUIO ECAT driver available!\n"));
        dwRetVal = EC_E_INVALIDSTATE;
        goto Exit;
    }
    else if (hNdisUioDevice != EC_NULL)
    {
        /* close handle, it was just to check if driver available */
        CloseHandle(hNdisUioDevice);
        hNdisUioDevice = EC_NULL;
    }
    /* NdisUio uses network adapter name to select appropriate network interface */
#if (defined UNICODE)
    _snwprintf((wchar_t*)pLinkParmsAdapter->szNetworkAdapterName, MAX_LEN_NDISUIO_ADAPTER_NAME, L"%S", *ptcWord);
#else
    _snwprintf(pLinkParmsAdapter->szNetworkAdapterName, MAX_LEN_NDISUIO_ADAPTER_NAME, L"%s", *ptcWord);
#endif
    /* no errors */
    *ppLinkParms = &pLinkParmsAdapter->linkParms;
    dwRetVal = EC_E_NOERROR;

Exit:
    if (EC_E_NOERROR != dwRetVal)
    {
        SafeOsFree(pLinkParmsAdapter);
    }
    return dwRetVal;
}
#endif /* INCLUDE_EMLLNDISUIO */

#if (defined INCLUDE_EMLLPROXY)
/***************************************************************************************************/
/**
\brief  Create Proxy link layer parameters according to current command line parsing

\return EC_E_NOERROR     if link layer parameters was created
EC_E_NOTFOUND    if command line was not matching
EC_E_INVALIDPARM if syntax error
*/
static EC_T_DWORD CreateLinkParmsFromCmdLineProxy(EC_T_CHAR** ptcWord, EC_T_CHAR** lpCmdLine, EC_T_CHAR* tcStorage, EC_T_BOOL* pbGetNextWord,
    EC_T_LINK_PARMS** ppLinkParms)
{
    EC_T_DWORD dwRetVal = EC_E_ERROR;
    EC_T_LINK_PARMS_PROXY* pLinkParmsAdapter = EC_NULL;
    EC_T_BOOL  bGetNextWord = EC_TRUE;

    EC_UNREFPARM(pbGetNextWord);

    /* check for matching adapter */
    if (OsStricmp(*ptcWord, "-proxy") != 0)
    {
        dwRetVal = EC_E_NOTFOUND;
        goto Exit;
    }
    /* alloc adapter specific link parms */
    pLinkParmsAdapter = (EC_T_LINK_PARMS_PROXY*)OsMalloc(sizeof(EC_T_LINK_PARMS_PROXY));
    if (EC_NULL == pLinkParmsAdapter)
    {
        dwRetVal = EC_E_NOMEMORY;
        goto Exit;
    }
    OsMemset(pLinkParmsAdapter, 0, sizeof(EC_T_LINK_PARMS_PROXY));
    LinkParmsInit(&pLinkParmsAdapter->linkParms, EC_LINK_PARMS_SIGNATURE_PROXY, sizeof(EC_T_LINK_PARMS_PROXY), EC_LINK_PARMS_IDENT_PROXY, 1, EcLinkMode_POLLING);

    /* parse mandatory parameters: instance, mode */
    if (!ParseDword(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.dwInstance)
        || !ParseLinkMode(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.eLinkMode))
    {
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }

    /* emrassocktype_udp */
    pLinkParmsAdapter->dwSocketType = 2;

    /* parse source IP address */
    *ptcWord = OsStrtok(EC_NULL, ".");
    if ((*ptcWord == EC_NULL) || (OsStrncmp(*ptcWord, "-", 1) == 0))
    {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "CreateLinkParmsFromCmdLineProxy: Source IP address missing!\n"));
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }
    if (!ParseIpAddress(*ptcWord, pLinkParmsAdapter->abySrcIpAddress))
    {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "CreateLinkParmsFromCmdLineProxy: Error parsing source IP address!\n"));
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }
    /* parse source port */
    if (!ParseWord(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->wSrcPort))
    {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "CreateLinkParmsFromCmdLineProxy: Error parsing source port number!\n"));
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }
    /* parse destination IP address */
    *ptcWord = OsStrtok(EC_NULL, ".");
    if ((*ptcWord == EC_NULL) || (OsStrncmp(*ptcWord, "-", 1) == 0))
    {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "CreateLinkParmsFromCmdLineProxy: Destination IP address missing!\n"));
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }
    if (!ParseIpAddress(*ptcWord, pLinkParmsAdapter->abyDstIpAddress))
    {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "CreateLinkParmsFromCmdLineProxy: Source Error parsing destination IP address!\n"));
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }
    /* parse destination port */
    if (!ParseWord(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->wDstPort))
    {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "CreateLinkParmsFromCmdLineProxy: Error parsing source port number!\n"));
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }

    /* get optional parameters */
    for (*ptcWord = OsStrtok(EC_NULL, " "); *ptcWord != EC_NULL;)
    {
        if (0 == OsStricmp(*ptcWord, "--mac"))
        {
            *ptcWord = OsStrtok(EC_NULL, " ");
            if (!ParseMacAddress(*ptcWord, pLinkParmsAdapter->abyMac))
            {
                EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "CreateLinkParmsFromCmdLineProxy: Error parsing MAC address!\n"));
                dwRetVal = EC_E_INVALIDPARM;
                goto Exit;
            }
        }
        else if (0 == OsStricmp(*ptcWord, "--rxbuffercnt"))
        {
            EC_T_DWORD dwRxBufferCnt = 0;
            *ptcWord = OsStrtok(EC_NULL, " ");
            if (!ParseDword(ptcWord, lpCmdLine, tcStorage, &dwRxBufferCnt))
            {
                EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "CreateLinkParmsFromCmdLineProxy: Error parsing RX buffer count!\n"));
                dwRetVal = EC_E_INVALIDPARM;
                goto Exit;
            }
            pLinkParmsAdapter->dwRxBufferCnt = dwRxBufferCnt;
        }

        else
        {
            bGetNextWord = EC_FALSE;
            break;
        }
        /* get next word */
        if (bGetNextWord)
        {
            *ptcWord = OsStrtok(EC_NULL, " ");
        }
        bGetNextWord = EC_TRUE;
    }

    /* no errors */
    *ppLinkParms = &pLinkParmsAdapter->linkParms;
    dwRetVal = EC_E_NOERROR;
Exit:
    if (EC_NULL != pbGetNextWord)
    {
        *pbGetNextWord = bGetNextWord;
    }
    if (EC_E_NOERROR != dwRetVal)
    {
        SafeOsFree(pLinkParmsAdapter);
    }
    return dwRetVal;
}
#endif /* INCLUDE_EMLLPROXY */

#if (defined INCLUDE_EMLLR6040)
/***************************************************************************************************/
/**
\brief  Create R6040 link layer parameters according to current command line parsing

\return EC_E_NOERROR     if link layer parameters was created
        EC_E_NOTFOUND    if command line was not matching
        EC_E_INVALIDPARM if syntax error
*/
static EC_T_DWORD CreateLinkParmsFromCmdLineR6040(EC_T_CHAR** ptcWord, EC_T_CHAR** lpCmdLine, EC_T_CHAR* tcStorage, EC_T_BOOL* pbGetNextWord,
                                                  EC_T_LINK_PARMS** ppLinkParms)
{
EC_T_DWORD dwRetVal = EC_E_ERROR;
EC_T_LINK_PARMS_R6040* pLinkParmsAdapter = EC_NULL;

    EC_UNREFPARM(pbGetNextWord);

    /* check for matching adapter */
    if (OsStricmp(*ptcWord, "-r6040") != 0)
    {
        dwRetVal = EC_E_NOTFOUND;
        goto Exit;
    }
    /* alloc adapter specific link parms */
    pLinkParmsAdapter = (EC_T_LINK_PARMS_R6040*)OsMalloc(sizeof(EC_T_LINK_PARMS_R6040));
    if (EC_NULL == pLinkParmsAdapter)
    {
        dwRetVal = EC_E_NOMEMORY;
        goto Exit;
    }
    OsMemset(pLinkParmsAdapter, 0, sizeof(EC_T_LINK_PARMS_R6040));
    LinkParmsInit(&pLinkParmsAdapter->linkParms, EC_LINK_PARMS_SIGNATURE_R6040, sizeof(EC_T_LINK_PARMS_R6040), EC_LINK_PARMS_IDENT_R6040, 1, EcLinkMode_POLLING);

    /* parse mandatory parameters: instance, mode */
    if ( !ParseDword(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.dwInstance)
        || !ParseLinkMode(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.eLinkMode))
    {
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }
    /* no errors */
    *ppLinkParms = &pLinkParmsAdapter->linkParms;
    dwRetVal = EC_E_NOERROR;

Exit:
    if (EC_E_NOERROR != dwRetVal)
    {
        SafeOsFree(pLinkParmsAdapter);
    }
    return dwRetVal;
}
#endif /* INCLUDE_EMLLR6040 */

#if (defined INCLUDE_EMLLRIN32)
/***************************************************************************************************/
/**
\brief  Try to create RIN32 link layer parameters according to current command line parsing

\return EC_E_NOERROR     if link layer parameters was created
        EC_E_NOTFOUND    if command line was not matching
        EC_E_INVALIDPARM if syntax error
*/
static EC_T_DWORD CreateLinkParmsFromCmdLineRIN32(EC_T_CHAR** ptcWord, EC_T_CHAR** lpCmdLine, EC_T_CHAR* tcStorage, EC_T_BOOL* pbGetNextWord,
                                                  EC_T_LINK_PARMS** ppLinkParms)
{
EC_T_DWORD dwRetVal = EC_E_ERROR;
EC_T_LINK_PARMS_RIN32* pLinkParmsAdapter = EC_NULL;
const size_t nParmsSize = sizeof(EC_T_LINK_PARMS_RIN32);

    EC_UNREFPARM(lpCmdLine);
    EC_UNREFPARM(tcStorage);
    EC_UNREFPARM(pbGetNextWord);

    /* check for matching adapter */
    if (OsStricmp(*ptcWord, "-rin32") != 0)
    {
        dwRetVal = EC_E_NOTFOUND;
        goto Exit;
    }
    /* alloc adapter specific link parms */
    pLinkParmsAdapter = (EC_T_LINK_PARMS_RIN32*)OsMalloc(nParmsSize);
    if (EC_NULL == pLinkParmsAdapter)
    {
        dwRetVal = EC_E_NOMEMORY;
        goto Exit;
    }
    OsMemset(pLinkParmsAdapter, 0, nParmsSize);
    LinkParmsInit(&pLinkParmsAdapter->linkParms, EC_LINK_PARMS_SIGNATURE_RIN32, nParmsSize, EC_LINK_PARMS_IDENT_RIN32, 1, EcLinkMode_POLLING);

    /* parse mandatory parameters: instance, mode */
    if ( !ParseDword(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.dwInstance)
        || !ParseLinkMode(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.eLinkMode))
    {
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }

    /* no errors */
    *ppLinkParms = &pLinkParmsAdapter->linkParms;
    dwRetVal = EC_E_NOERROR;

Exit:
    if (EC_E_NOERROR != dwRetVal)
    {
        SafeOsFree(pLinkParmsAdapter);
    }
    return dwRetVal;
}
#endif /* EC_VERSION_RIN32 */

#if (defined INCLUDE_EMLLRTL8139)
/***************************************************************************************************/
/**
\brief  Create RTL8139 link layer parameters according to current command line parsing

\return EC_E_NOERROR     if link layer parameters was created
        EC_E_NOTFOUND    if command line was not matching
        EC_E_INVALIDPARM if syntax error
*/
static EC_T_DWORD CreateLinkParmsFromCmdLineRTL8139(EC_T_CHAR** ptcWord, EC_T_CHAR** lpCmdLine, EC_T_CHAR* tcStorage, EC_T_BOOL* pbGetNextWord,
                                                    EC_T_LINK_PARMS** ppLinkParms)
{
EC_T_DWORD dwRetVal = EC_E_ERROR;
EC_T_LINK_PARMS_RTL8139* pLinkParmsAdapter = EC_NULL;

    EC_UNREFPARM(pbGetNextWord);

    /* check for matching adapter */
    if (OsStricmp(*ptcWord, "-rtl8139") != 0)
    {
        dwRetVal = EC_E_NOTFOUND;
        goto Exit;
    }
    /* alloc adapter specific link parms */
    pLinkParmsAdapter = (EC_T_LINK_PARMS_RTL8139*)OsMalloc(sizeof(EC_T_LINK_PARMS_RTL8139));
    if (EC_NULL == pLinkParmsAdapter)
    {
        dwRetVal = EC_E_NOMEMORY;
        goto Exit;
    }
    OsMemset(pLinkParmsAdapter, 0, sizeof(EC_T_LINK_PARMS_RTL8139));
    LinkParmsInit(&pLinkParmsAdapter->linkParms, EC_LINK_PARMS_SIGNATURE_RTL8139, sizeof(EC_T_LINK_PARMS_RTL8139), EC_LINK_PARMS_IDENT_RTL8139, 1, EcLinkMode_POLLING);

    /* parse mandatory parameters: instance, mode */
    if ( !ParseDword(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.dwInstance)
        || !ParseLinkMode(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.eLinkMode))
    {
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }
    /* no errors */
    *ppLinkParms = &pLinkParmsAdapter->linkParms;
    dwRetVal = EC_E_NOERROR;

Exit:
    if (EC_E_NOERROR != dwRetVal)
    {
        SafeOsFree(pLinkParmsAdapter);
    }
    return dwRetVal;
}
#endif /* INCLUDE_EMLLRTL8139 */

#if (defined INCLUDE_EMLLRTL8169)
/***************************************************************************************************/
/**
\brief  Create RTL8169 link layer parameters according to current command line parsing

\return EC_E_NOERROR     if link layer parameters was created
        EC_E_NOTFOUND    if command line was not matching
        EC_E_INVALIDPARM if syntax error
*/
static EC_T_DWORD CreateLinkParmsFromCmdLineRTL8169(EC_T_CHAR** ptcWord, EC_T_CHAR** lpCmdLine, EC_T_CHAR* tcStorage, EC_T_BOOL* pbGetNextWord,
                                                    EC_T_LINK_PARMS** ppLinkParms)
{
EC_T_DWORD dwRetVal = EC_E_ERROR;
EC_T_LINK_PARMS_RTL8169* pLinkParmsAdapter = EC_NULL;

    EC_UNREFPARM(pbGetNextWord);

    /* check for matching adapter */
    if (OsStricmp(*ptcWord, "-rtl8169") != 0)
    {
        dwRetVal = EC_E_NOTFOUND;
        goto Exit;
    }
    /* alloc adapter specific link parms */
    pLinkParmsAdapter = (EC_T_LINK_PARMS_RTL8169*)OsMalloc(sizeof(EC_T_LINK_PARMS_RTL8169));
    if (EC_NULL == pLinkParmsAdapter)
    {
        dwRetVal = EC_E_NOMEMORY;
        goto Exit;
    }
    OsMemset(pLinkParmsAdapter, 0, sizeof(EC_T_LINK_PARMS_RTL8169));
    LinkParmsInit(&pLinkParmsAdapter->linkParms, EC_LINK_PARMS_SIGNATURE_RTL8169, sizeof(EC_T_LINK_PARMS_RTL8169), EC_LINK_PARMS_IDENT_RTL8169, 1, EcLinkMode_POLLING);

    if (!ParseDword(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.dwInstance)
       || !ParseLinkMode(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.eLinkMode))
    {
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }

    pLinkParmsAdapter->bNotUseDmaBuffers = EC_FALSE;
#if (defined _ARM_) || (defined __arm__) || (defined __aarch64__)
    /* for arm platform we should not use DMA memory because any unaligned access creates crash */
    pLinkParmsAdapter->bNotUseDmaBuffers = EC_TRUE;
#endif
    pLinkParmsAdapter->bNoPhyAccess = EC_FALSE;

    /* no errors */
    *ppLinkParms = &pLinkParmsAdapter->linkParms;
    dwRetVal = EC_E_NOERROR;

Exit:
    if (EC_E_NOERROR != dwRetVal)
    {
        SafeOsFree(pLinkParmsAdapter);
    }
    return dwRetVal;
}
#endif /* INCLUDE_EMLLRTL8169 */

#if (defined INCLUDE_EMLLRZT1)
/***************************************************************************************************/
/**
\brief  Create RZT1 link layer parameters according to current command line parsing

\return EC_E_NOERROR     if link layer parameters was created
EC_E_NOTFOUND    if command line was not matching
EC_E_INVALIDPARM if syntax error
*/
static EC_T_DWORD CreateLinkParmsFromCmdLineRZT1(EC_T_CHAR** ptcWord, EC_T_CHAR** lpCmdLine, EC_T_CHAR* tcStorage, EC_T_BOOL* pbGetNextWord,
    EC_T_LINK_PARMS** ppLinkParms)
{
    EC_T_DWORD dwRetVal = EC_E_ERROR;
    EC_T_LINK_PARMS_RZT1* pLinkParmsAdapter = EC_NULL;
    const size_t nParmsSize = sizeof(EC_T_LINK_PARMS_RZT1);

    EC_UNREFPARM(lpCmdLine);
    EC_UNREFPARM(tcStorage);
    EC_UNREFPARM(pbGetNextWord);

    /* check for matching adapter */
    if (OsStricmp(*ptcWord, "-rzt1") != 0)
    {
        dwRetVal = EC_E_NOTFOUND;
        goto Exit;
    }
    /* alloc adapter specific link parms */
    pLinkParmsAdapter = (EC_T_LINK_PARMS_RZT1*)OsMalloc(nParmsSize);
    if (EC_NULL == pLinkParmsAdapter)
    {
        dwRetVal = EC_E_NOMEMORY;
        goto Exit;
    }
    OsMemset(pLinkParmsAdapter, 0, nParmsSize);
    LinkParmsInit(&pLinkParmsAdapter->linkParms, EC_LINK_PARMS_SIGNATURE_RZT1, nParmsSize, EC_LINK_PARMS_IDENT_RZT1, 1, EcLinkMode_POLLING);

    /* parse mandatory parameters */
    if (!ParseDword(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.dwInstance))
    {
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }

    if (pLinkParmsAdapter->linkParms.dwInstance < 1 || pLinkParmsAdapter->linkParms.dwInstance > 2)
    {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Instance number must be 1 or 2\n"));
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }

    /* no errors */
    *ppLinkParms = &pLinkParmsAdapter->linkParms;
    dwRetVal = EC_E_NOERROR;

Exit:
    if (EC_E_NOERROR != dwRetVal)
    {
        SafeOsFree(pLinkParmsAdapter);
    }
    return dwRetVal;
}
#endif /* EC_VERSION_RZT1 */

#if (defined INCLUDE_EMLLSHETH)
/***************************************************************************************************/
/**
\brief  Create Super H link layer parameters according to current command line parsing

\return EC_E_NOERROR     if link layer parameters was created
EC_E_NOTFOUND    if command line was not matching
EC_E_INVALIDPARM if syntax error
*/
static EC_T_DWORD CreateLinkParmsFromCmdLineSHEth(EC_T_CHAR** ptcWord, EC_T_CHAR** lpCmdLine, EC_T_CHAR* tcStorage, EC_T_BOOL* pbGetNextWord,
    EC_T_LINK_PARMS** ppLinkParms)
{
    EC_T_DWORD dwRetVal = EC_E_ERROR;
    EC_T_LINK_PARMS_SHETH* pLinkParmsAdapter = EC_NULL;

    EC_UNREFPARM(pbGetNextWord);

    /* check for matching adapter */
    if (OsStricmp(*ptcWord, "-sheth") != 0)
    {
        dwRetVal = EC_E_NOTFOUND;
        goto Exit;
    }
    /* get next word */
    *ptcWord = GetNextWord(lpCmdLine, tcStorage);
    if ((*ptcWord == EC_NULL) || (OsStrncmp(*ptcWord, "-", 1) == 0))
    {
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }
    /* alloc adapter specific link parms */
    pLinkParmsAdapter = (EC_T_LINK_PARMS_SHETH*)OsMalloc(sizeof(EC_T_LINK_PARMS_SHETH));
    if (EC_NULL == pLinkParmsAdapter)
    {
        dwRetVal = EC_E_NOMEMORY;
        goto Exit;
    }
    OsMemset(pLinkParmsAdapter, 0, sizeof(EC_T_LINK_PARMS_SHETH));
    LinkParmsInit(&pLinkParmsAdapter->linkParms, EC_LINK_PARMS_SIGNATURE_SHETH, sizeof(EC_T_LINK_PARMS_SHETH), EC_LINK_PARMS_IDENT_SHETH, 1, EcLinkMode_POLLING);

    /* parse mandatory parameters */
    if (!ParseDword(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.dwInstance))
    {
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }

    if (pLinkParmsAdapter->linkParms.dwInstance < 1 || pLinkParmsAdapter->linkParms.dwInstance > 2)
    {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Instance number must be 1 or 2\n"));
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }

    /* get reference board */
    *ptcWord = GetNextWord(lpCmdLine, tcStorage);
    if (0 == OsStricmp(*ptcWord, "a800eva"))
    {
        pLinkParmsAdapter->eType = eSHEth_R8A7740;
        pLinkParmsAdapter->dwBaseAddr = 0xE9A00000;
        pLinkParmsAdapter->byPhyAddr = 0;
    }
    else if (0 == OsStricmp(*ptcWord, "rzg1e"))
    {
        pLinkParmsAdapter->eType = eSHEth_R8A77450;
        pLinkParmsAdapter->dwBaseAddr = 0xEE700000;
        pLinkParmsAdapter->byPhyAddr = 1;
    }
    else
    {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Invalid RefBoard value\n"));
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }

    {
        EC_T_BYTE abyStationAddress[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xBA, 0xBE };
        memcpy(pLinkParmsAdapter->abyStationAddress, abyStationAddress, sizeof(pLinkParmsAdapter->abyStationAddress));
    }

    pLinkParmsAdapter->bNotUseDmaBuffers = EC_TRUE;

    /* no errors */
    *ppLinkParms = &pLinkParmsAdapter->linkParms;
    dwRetVal = EC_E_NOERROR;

Exit:
    if (EC_E_NOERROR != dwRetVal)
    {
        SafeOsFree(pLinkParmsAdapter);
    }
    return dwRetVal;
}
#endif /* INCLUDE_EMLLSHETH */

#if (defined INCLUDE_EMLLSIMULATOR)
EC_T_DWORD CreateLinkParmsSimulator(EC_T_LINK_PARMS_SIMULATOR** ppLinkParmsAdapter)
{
    EC_T_DWORD dwRetVal = EC_E_ERROR;

    /* alloc adapter specific link parms */
    *ppLinkParmsAdapter = (EC_T_LINK_PARMS_SIMULATOR*)OsMalloc(sizeof(EC_T_LINK_PARMS_SIMULATOR));
    if (EC_NULL == *ppLinkParmsAdapter)
    {
        dwRetVal = EC_E_NOMEMORY;
        goto Exit;
    }
    OsMemset(*ppLinkParmsAdapter, 0, sizeof(EC_T_LINK_PARMS_SIMULATOR));
    LinkParmsInit(&((*ppLinkParmsAdapter)->linkParms), EC_LINK_PARMS_SIGNATURE_SIMULATOR, sizeof(EC_T_LINK_PARMS_SIMULATOR), EC_LINK_PARMS_IDENT_SIMULATOR, 1, EcLinkMode_POLLING);
    ((EC_T_LINK_PARMS_SIMULATOR*)*ppLinkParmsAdapter)->bJobsExecutedByApp = EC_FALSE;
    ((EC_T_LINK_PARMS_SIMULATOR*)*ppLinkParmsAdapter)->bConnectHcGroups = EC_TRUE;
    ((EC_T_LINK_PARMS_SIMULATOR*)*ppLinkParmsAdapter)->eCnfType = eCnfType_Filename;

    /* no errors */
    dwRetVal = EC_E_NOERROR;

Exit:
    if (EC_E_NOERROR != dwRetVal)
    {
        SafeOsFree(*ppLinkParmsAdapter);
    }
    return dwRetVal;
}
/***************************************************************************************************/
/**
\brief  Create Simulator link layer parameters according to current command line parsing

\return EC_E_NOERROR     if link layer parameters was created
EC_E_NOTFOUND    if command line was not matching
EC_E_INVALIDPARM if syntax error
*/
static EC_T_DWORD CreateLinkParmsFromCmdLineSimulator(EC_T_CHAR** ptcWord, EC_T_CHAR** lpCmdLine, EC_T_CHAR* tcStorage, EC_T_BOOL* pbGetNextWord,
    EC_T_LINK_PARMS** ppLinkParms)
{
    EC_T_DWORD dwRetVal     = EC_E_ERROR;
    EC_T_DWORD dwRes        = EC_E_ERROR;
    EC_T_BOOL  bGetNextWord = EC_TRUE;
    EC_T_INT   nPtcWordIdx  = 0;
    EC_T_LINK_PARMS_SIMULATOR* pLinkParmsAdapter = EC_NULL;

    /* check for matching adapter */
    if (OsStricmp(*ptcWord, "-simulator") != 0)
    {
        dwRetVal = EC_E_NOTFOUND;
        goto Exit;
    }
    /* alloc adapter specific link parms */
    CreateLinkParmsSimulator(&pLinkParmsAdapter);
    if (EC_NULL == pLinkParmsAdapter)
    {
        dwRetVal = EC_E_NOMEMORY;
        goto Exit;
    }

    /* parse mandatory parameters: instance, mode */
    if (!ParseDword(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.dwInstance)
        || !ParseLinkMode(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.eLinkMode))
    {
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }
    /* get ENI file name */
    if (!ParseString(ptcWord, lpCmdLine, tcStorage))
    {
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }

    pLinkParmsAdapter->eCnfType = eCnfType_Filename;
    pLinkParmsAdapter->dwCnfDataLen = (EC_T_DWORD)OsStrlen(*ptcWord);
    pLinkParmsAdapter->pbyCnfData = (EC_T_BYTE*)OsMalloc(pLinkParmsAdapter->dwCnfDataLen + 1);
    if (EC_NULL == pLinkParmsAdapter->pbyCnfData)
    {
        dwRetVal = EC_E_NOMEMORY;
        goto Exit;
    }
    OsSnprintf((EC_T_CHAR*)pLinkParmsAdapter->pbyCnfData, pLinkParmsAdapter->dwCnfDataLen + 1, "%s", *ptcWord);
    pLinkParmsAdapter->pbyCnfData[pLinkParmsAdapter->dwCnfDataLen] = '\0';

    /* distinguish between main and red adapter */
    pLinkParmsAdapter->abyMac[5] = (EC_T_BYTE)pLinkParmsAdapter->linkParms.dwInstance;

    /* get optional parameters */
    for (*ptcWord = OsStrtok(EC_NULL, " "); *ptcWord != EC_NULL;)
    {
        if (0 == OsStricmp(*ptcWord, "--connect"))
        {
            /* no slaves hosted at this EC-Simulator instance */
            pLinkParmsAdapter->eCnfType = eCnfType_None;
            SafeOsFree(pLinkParmsAdapter->pbyCnfData);
            pLinkParmsAdapter->dwCnfDataLen = 0;
            pLinkParmsAdapter->bJobsExecutedByApp = EC_FALSE;

            *ptcWord = OsStrtok(EC_NULL, " ");

            /* e.g. --connect slave 1 1001 2 */
            if (OsStrncmp(*ptcWord, "slave", 6) == 0)
            {
                pLinkParmsAdapter->oDeviceConnection.dwType = EC_SIMULATOR_DEVICE_CONNECTION_TYPE_SLAVE;
                *ptcWord = OsStrtok(EC_NULL, " ");
                if (OsStrncmp(*ptcWord, "-", 1) == 0)
                {
                    dwRetVal = EC_E_INVALIDPARM;
                    goto Exit;
                }
                pLinkParmsAdapter->oDeviceConnection.dwInstanceID = (EC_T_DWORD)OsStrtol(*ptcWord, EC_NULL, 10);
                *ptcWord = OsStrtok(EC_NULL, " ");
                if (OsStrncmp(*ptcWord, "-", 1) == 0)
                {
                    dwRetVal = EC_E_INVALIDPARM;
                    goto Exit;
                }
                pLinkParmsAdapter->oDeviceConnection.wCfgFixedAddress = (EC_T_WORD)OsStrtol(*ptcWord, EC_NULL, 10);
                *ptcWord = OsStrtok(EC_NULL, " ");
                if (OsStrncmp(*ptcWord, "-", 1) == 0)
                {
                    dwRetVal = EC_E_INVALIDPARM;
                    goto Exit;
                }
                pLinkParmsAdapter->oDeviceConnection.byPort = (EC_T_BYTE)OsStrtol(*ptcWord, EC_NULL, 10);
            }
            else
            {
                dwRetVal = EC_E_INVALIDPARM;
                goto Exit;
            }
            *ptcWord = OsStrtok(EC_NULL, " ");
        }

        else if (0 == OsStricmp(*ptcWord, "--mac"))
        {
            *ptcWord = OsStrtok(EC_NULL, " ");
            if (!ParseMacAddress(*ptcWord, pLinkParmsAdapter->abyMac))
            {
                EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "CreateLinkParmsFromCmdLineSimulator: Error parsing MAC address!\n"));
                dwRetVal = EC_E_INVALIDPARM;
                goto Exit;
            }
        }

        else if (0 == OsStricmp(*ptcWord, "--oem"))
        {
            *ptcWord = OsStrtok(EC_NULL, " ");
            if (ptcWord == EC_NULL)
            {
                dwRetVal = EC_E_INVALIDPARM;
                goto Exit;
            }
            pLinkParmsAdapter->qwOemKey = EcStrtoull(*ptcWord, EC_NULL, 0);
            if (UINTMAX_MAX <= pLinkParmsAdapter->qwOemKey)
            {
                dwRetVal = EC_E_INVALIDPARM;
                goto Exit;
            }
        }

        else if (0 == OsStricmp(*ptcWord, "--lic"))
        {
            /* search license string */
            for (nPtcWordIdx = 5; (*ptcWord)[nPtcWordIdx] != '\0'; nPtcWordIdx++)
            {
                if (*ptcWord[nPtcWordIdx] != ' ')
                {
                    break;
                }
            }
            /* extract license string */
            *ptcWord = OsStrtok(EC_NULL, " ");
            if ((*ptcWord == EC_NULL) || (OsStrncmp(*ptcWord, "-", 1) == 0))
            {
                dwRetVal = EC_E_INVALIDPARM;
                goto Exit;
            }
            if (OsStrlen(*ptcWord) > EC_SIMULATOR_KEY_MAXLEN)
            {
                EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "CreateLinkParmsFromCmdLineSimulator: invalid license key %s!\n", *ptcWord));
                dwRetVal = EC_E_INVALIDPARM;
                goto Exit;
            }
            OsSnprintf(pLinkParmsAdapter->szLicenseKey, EC_SIMULATOR_KEY_MAXLEN, "%s", *ptcWord);
        }

        else if (0 == OsStricmp(*ptcWord, "--link"))
        {
            /* parse link layer parms for MAC validation of key */
            EC_T_LINK_PARMS* pLinkParms = EC_NULL;
            *ptcWord = OsStrtok(EC_NULL, " ");
            dwRes = CreateLinkParmsFromCmdLine(ptcWord, lpCmdLine, tcStorage, &bGetNextWord, &pLinkParms, EC_NULL);
            if (EC_E_NOERROR != dwRes)
            {
                EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "CreateLinkParmsFromCmdLineSimulator: error reading link parameters of network adapter matching MAC address of license key!\n", *ptcWord));
                dwRetVal = dwRes;
                goto Exit;
            }
            pLinkParmsAdapter->apLinkParms[0] = pLinkParms;
        }
#if (defined INCLUDE_RAS_SERVER)
        else if (0 == OsStricmp(*ptcWord, "--sp"))
        {
            pLinkParmsAdapter->bStartRasServer = EC_TRUE;

            *ptcWord = OsStrtok(EC_NULL, " ");
            if ((*ptcWord == NULL) || (OsStrncmp(*ptcWord, "-", 1) == 0))
            {
                /* optional sub parameter not found, use current word for next parameter */
                bGetNextWord = EC_FALSE;
            }
            else
            {
                pLinkParmsAdapter->wRasServerPort = (EC_T_WORD)OsStrtol(*ptcWord, EC_NULL, 10);
            }
            pLinkParmsAdapter->dwRasPriority = MAIN_THREAD_PRIO;
        }
#endif
        else
        {
            bGetNextWord = EC_FALSE;
            break;
        }
        /* get next word */
        if (bGetNextWord)
        {
            *ptcWord = OsStrtok(EC_NULL, " ");
        }
        bGetNextWord = EC_TRUE;
    }
    /* no errors */
    *ppLinkParms = &pLinkParmsAdapter->linkParms;
    dwRetVal = EC_E_NOERROR;

Exit:
    if (EC_NULL != pbGetNextWord)
    {
        *pbGetNextWord = bGetNextWord;
    }
    if (EC_E_NOERROR != dwRetVal)
    {
        SafeOsFree(pLinkParmsAdapter);
    }
    return dwRetVal;
}
#endif /* INCLUDE_EMLLSIMULATOR */

#if (defined INCLUDE_EMLLSNARF)
/***************************************************************************************************/
/**
\brief  Create Snarf link layer parameters according to current command line parsing

\return EC_E_NOERROR     if link layer parameters was created
        EC_E_NOTFOUND    if command line was not matching
        EC_E_INVALIDPARM if syntax error
*/
static EC_T_DWORD CreateLinkParmsFromCmdLineSnarf(EC_T_CHAR** ptcWord, EC_T_CHAR** lpCmdLine, EC_T_CHAR* tcStorage, EC_T_BOOL* pbGetNextWord,
                                                  EC_T_LINK_PARMS** ppLinkParms)
{
EC_T_DWORD dwRetVal = EC_E_ERROR;
EC_T_LINK_PARMS_SNARF* pLinkParmsAdapter = EC_NULL;

    EC_UNREFPARM(pbGetNextWord);

    /* check for matching adapter */
    if (OsStricmp(*ptcWord, "-snarf") != 0)
    {
        dwRetVal = EC_E_NOTFOUND;
        goto Exit;
    }
    /* get next word */
    *ptcWord = GetNextWord(lpCmdLine, tcStorage);
    if ((*ptcWord == EC_NULL) || (OsStrncmp(*ptcWord, "-", 1) == 0))
    {
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }
    /* alloc adapter specific link parms */
    pLinkParmsAdapter = (EC_T_LINK_PARMS_SNARF*)OsMalloc(sizeof(EC_T_LINK_PARMS_SNARF));
    if (EC_NULL == pLinkParmsAdapter)
    {
        dwRetVal = EC_E_NOMEMORY;
        goto Exit;
    }
    OsMemset(pLinkParmsAdapter, 0, sizeof(EC_T_LINK_PARMS_SNARF));
    LinkParmsInit(&pLinkParmsAdapter->linkParms, EC_LINK_PARMS_SIGNATURE_SNARF, sizeof(EC_T_LINK_PARMS_SNARF), EC_LINK_PARMS_IDENT_SNARF, 1, EcLinkMode_POLLING);

    /* get adapter name */
    OsSnprintf(pLinkParmsAdapter->szAdapterName, MAX_LEN_SNARF_ADAPTER_NAME, "%s", *ptcWord);

    /* no errors */
    *ppLinkParms = &pLinkParmsAdapter->linkParms;
    dwRetVal = EC_E_NOERROR;

Exit:
    if (EC_E_NOERROR != dwRetVal)
    {
        SafeOsFree(pLinkParmsAdapter);
    }
    return dwRetVal;
}
#endif /* INCLUDE_EMLLSNARF */

#if (defined INCLUDE_EMLLSOCKRAW)
/***************************************************************************************************/
/**
\brief  Create SockRaw link layer parameters according to current command line parsing

\return EC_E_NOERROR     if link layer parameters was created
        EC_E_NOTFOUND    if command line was not matching
        EC_E_INVALIDPARM if syntax error
*/
static EC_T_DWORD CreateLinkParmsFromCmdLineSockRaw(EC_T_CHAR** ptcWord, EC_T_CHAR** lpCmdLine, EC_T_CHAR* tcStorage, EC_T_BOOL* pbGetNextWord,
                                                    EC_T_LINK_PARMS** ppLinkParms)
{
EC_T_DWORD dwRetVal = EC_E_ERROR;
EC_T_LINK_PARMS_SOCKRAW* pLinkParmsAdapter = EC_NULL;

    EC_UNREFPARM(pbGetNextWord);

    /* check for matching adapter */
    if (OsStricmp(*ptcWord, "-sockraw") != 0)
    {
        dwRetVal = EC_E_NOTFOUND;
        goto Exit;
    }
    /* alloc adapter specific link parms */
    pLinkParmsAdapter = (EC_T_LINK_PARMS_SOCKRAW*)OsMalloc(sizeof(EC_T_LINK_PARMS_SOCKRAW));
    if (EC_NULL == pLinkParmsAdapter)
    {
        dwRetVal = EC_E_NOMEMORY;
        goto Exit;
    }
    OsMemset(pLinkParmsAdapter, 0, sizeof(EC_T_LINK_PARMS_SOCKRAW));
    LinkParmsInit(&pLinkParmsAdapter->linkParms, EC_LINK_PARMS_SIGNATURE_SOCKRAW, sizeof(EC_T_LINK_PARMS_SOCKRAW), EC_LINK_PARMS_IDENT_SOCKRAW, 1, EcLinkMode_POLLING);

    /* get adapter name */
    *ptcWord = GetNextWord(lpCmdLine, tcStorage);
    if ((*ptcWord == EC_NULL) || (OsStrncmp(*ptcWord, "-", 1) == 0))
    {
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }
    if (OsStrlen((char*)*ptcWord) > EC_SOCKRAW_ADAPTER_NAME_MAXLEN)
    {
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }
    OsStrncpy(pLinkParmsAdapter->szAdapterName, (char*)*ptcWord, EC_SOCKRAW_ADAPTER_NAME_MAXLEN);

#if (defined DISABLE_FORCE_BROADCAST)
    /* Do not overwrite destination in frame with FF:FF:FF:FF:FF:FF, needed for EAP. */
    pLinkParmsAdapter->bDisableForceBroadcast = EC_TRUE;
#endif

    /* enhance receive performance */
    pLinkParmsAdapter->bUsePacketMmapRx = EC_TRUE;

#if 0
    /* needed for TI CPSW */
    pLinkParmsAdapter->bReplacePaddingWithNopCmd =  EC_TRUE;
#endif

    /* get optional parameters */
    for (*ptcWord = OsStrtok(EC_NULL, " "); *ptcWord != EC_NULL;)
    {
        if (0 == OsStricmp(*ptcWord, "--nommaprx"))
        {
            pLinkParmsAdapter->bUsePacketMmapRx = EC_FALSE;
        }
        else if (('0' == *ptcWord[0]) || ('1' == *ptcWord[0]))
        {
            pLinkParmsAdapter->linkParms.eLinkMode = ('0' == *ptcWord[0]) ? EcLinkMode_INTERRUPT : EcLinkMode_POLLING;
        }
        else
        {
            *pbGetNextWord = EC_FALSE;
            break;
        }

        *ptcWord = OsStrtok(EC_NULL, " ");
    }

    /* no errors */
    *ppLinkParms = &pLinkParmsAdapter->linkParms;
    dwRetVal = EC_E_NOERROR;

Exit:
    if (EC_E_NOERROR != dwRetVal)
    {
        SafeOsFree(pLinkParmsAdapter);
    }
    return dwRetVal;
}
#endif /* INCLUDE_EMLLSOCKRAW */

#if (defined INCLUDE_EMLLSTM32ETH)
/***************************************************************************************************/
/**
\brief  Create Stm2Eth link layer parameters according to current command line parsing

\return EC_E_NOERROR     if link layer parameters was created
        EC_E_NOTFOUND    if command line was not matching
        EC_E_INVALIDPARM if syntax error
*/
static EC_T_DWORD CreateLinkParmsFromCmdLineStm32Eth(
    EC_T_CHAR** ptcWord, EC_T_CHAR** lpCmdLine, EC_T_CHAR* tcStorage, EC_T_BOOL* pbGetNextWord,
    EC_T_LINK_PARMS** ppLinkParms)
{
EC_T_DWORD dwRetVal = EC_E_ERROR;
EC_T_LINK_PARMS_STM32ETH* pLinkParmsAdapter = EC_NULL;

    EC_UNREFPARM(pbGetNextWord);

    /* check for matching adapter */
    if (OsStricmp(*ptcWord, "-stm32eth") != 0)
    {
        dwRetVal = EC_E_NOTFOUND;
        goto Exit;
    }
    /* alloc adapter specific link parms */
    pLinkParmsAdapter = (EC_T_LINK_PARMS_STM32ETH*)OsMalloc(sizeof(EC_T_LINK_PARMS_STM32ETH));
    if (EC_NULL == pLinkParmsAdapter)
    {
        dwRetVal = EC_E_NOMEMORY;
        goto Exit;
    }
    OsMemset(pLinkParmsAdapter, 0, sizeof(EC_T_LINK_PARMS_STM32ETH));
    LinkParmsInit(&pLinkParmsAdapter->linkParms, EC_LINK_PARMS_SIGNATURE_STM32ETH,
        sizeof(EC_T_LINK_PARMS_STM32ETH), EC_LINK_PARMS_IDENT_STM32ETH, 1, EcLinkMode_POLLING);

    /* parse mandatory parameters: instance, mode */
    if (!ParseDword(ptcWord, lpCmdLine, tcStorage, (EC_T_DWORD*)&pLinkParmsAdapter->linkParms.dwInstance)
        || !ParseLinkMode(ptcWord, lpCmdLine, tcStorage, (EC_T_LINKMODE*)&pLinkParmsAdapter->linkParms.eLinkMode))
    {
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }

    /* no errors */
    *ppLinkParms = &pLinkParmsAdapter->linkParms;
    dwRetVal = EC_E_NOERROR;

Exit:
    if (EC_E_NOERROR != dwRetVal)
    {
        SafeOsFree(pLinkParmsAdapter);
    }
    return dwRetVal;
}
#endif /* INCLUDE_EMLLSTM32ETH */

#if (defined INCLUDE_EMLLTIENETICSSG)
/***************************************************************************************************/
/**
\brief  Create Ti Enet LLD Icssg link layer parameters according to current command line parsing

\return EC_E_NOERROR     if link layer parameters was created
        EC_E_NOTFOUND    if command line was not matching
        EC_E_INVALIDPARM if syntax error
*/
static EC_T_DWORD CreateLinkParmsFromCmdLineTiEnetIcssg(
    EC_T_CHAR** ptcWord, EC_T_CHAR** lpCmdLine, EC_T_CHAR* tcStorage, EC_T_BOOL* pbGetNextWord,
    EC_T_LINK_PARMS** ppLinkParms)
{
EC_T_DWORD dwRetVal = EC_E_ERROR;
EC_T_LINK_PARMS_TIENETICSSG* pLinkParmsAdapter = EC_NULL;

    EC_UNREFPARM(pbGetNextWord);

    /* check for matching adapter */
    if (OsStricmp(*ptcWord, "-TiEnetIcssg") != 0)
    {
        dwRetVal = EC_E_NOTFOUND;
        goto Exit;
    }
    /* alloc adapter specific link parms */
    pLinkParmsAdapter = (EC_T_LINK_PARMS_TIENETICSSG*)OsMalloc(sizeof(EC_T_LINK_PARMS_TIENETICSSG));
    if (EC_NULL == pLinkParmsAdapter)
    {
        dwRetVal = EC_E_NOMEMORY;
        goto Exit;
    }
    OsMemset(pLinkParmsAdapter, 0, sizeof(EC_T_LINK_PARMS_TIENETICSSG));
    LinkParmsInit(&pLinkParmsAdapter->linkParms, EC_LINK_PARMS_SIGNATURE_TIENETICSSG,
        sizeof(EC_T_LINK_PARMS_TIENETICSSG), EC_LINK_PARMS_IDENT_TIENETICSSG, 1, EcLinkMode_POLLING);

    /* parse mandatory parameters: instance, mode */
    if (!ParseDword(ptcWord, lpCmdLine, tcStorage, (EC_T_DWORD*)&pLinkParmsAdapter->linkParms.dwInstance)
        || !ParseLinkMode(ptcWord, lpCmdLine, tcStorage, (EC_T_LINKMODE*)&pLinkParmsAdapter->linkParms.eLinkMode))
    {
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }

    /* get master/slave flag */
    *ptcWord = GetNextWord(lpCmdLine, tcStorage);
    if ((0 != *ptcWord) && (0 == OsStricmp(*ptcWord, "m")))
    {
        pLinkParmsAdapter->bMaster = EC_TRUE;
    }
    else if ((0 != *ptcWord) && (0 == OsStricmp(*ptcWord, "s")))
    {
        pLinkParmsAdapter->bMaster = EC_FALSE;
    }
    else
    {
        EcLogMsg(EC_LOG_LEVEL_WARNING, (pEcLogContext, EC_LOG_LEVEL_WARNING, "ICSSG: No master/slave flag specified. Assume master\n"));
        pLinkParmsAdapter->bMaster = EC_TRUE;
        *pbGetNextWord = EC_FALSE;
    }

    /* no errors */
    *ppLinkParms = &pLinkParmsAdapter->linkParms;
    dwRetVal = EC_E_NOERROR;

Exit:
    if (EC_E_NOERROR != dwRetVal)
    {
        SafeOsFree(pLinkParmsAdapter);
    }
    return dwRetVal;
}
#endif /* INCLUDE_EMLLTIENETICSSG */

#if (defined INCLUDE_EMLLWINPCAP)
/***************************************************************************************************/
/**
\brief  Create WinPcap link layer parameters according to current command line parsing

\return EC_E_NOERROR     if link layer parameters was created
        EC_E_NOTFOUND    if command line was not matching
        EC_E_INVALIDPARM if syntax error
*/
static EC_T_DWORD CreateLinkParmsFromCmdLineWinPcap(EC_T_CHAR** ptcWord, EC_T_CHAR** lpCmdLine, EC_T_CHAR* tcStorage, EC_T_BOOL* pbGetNextWord,
                                                    EC_T_LINK_PARMS** ppLinkParms)
{
    EC_T_DWORD dwRetVal = EC_E_ERROR;
    EC_T_LINK_PARMS_WINPCAP* pLinkParmsAdapter = EC_NULL;

    EC_UNREFPARM(pbGetNextWord);

    /* check for matching adapter */
    if (OsStricmp(*ptcWord, "-winpcap") != 0)
    {
        dwRetVal = EC_E_NOTFOUND;
        goto Exit;
    }
    /* alloc adapter specific link parms */
    pLinkParmsAdapter = (EC_T_LINK_PARMS_WINPCAP*)OsMalloc(sizeof(EC_T_LINK_PARMS_WINPCAP));
    if (EC_NULL == pLinkParmsAdapter)
    {
        dwRetVal = EC_E_NOMEMORY;
        goto Exit;
    }
    OsMemset(pLinkParmsAdapter, 0, sizeof(EC_T_LINK_PARMS_WINPCAP));
    LinkParmsInit(&pLinkParmsAdapter->linkParms, EC_LINK_PARMS_SIGNATURE_WINPCAP, sizeof(EC_T_LINK_PARMS_WINPCAP), EC_LINK_PARMS_IDENT_WINPCAP, 1, EcLinkMode_POLLING);

    /* parse IP address */
    *ptcWord = OsStrtok(EC_NULL, ".");
    if ((*ptcWord == EC_NULL) || (OsStrncmp(*ptcWord, "-", 1) == 0))
    {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "CreateLinkParmsFromCmdLineWinPcap: IP address missing!\n"));
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }
    if (!ParseIpAddress(*ptcWord, pLinkParmsAdapter->abyIpAddress))
    {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "CreateLinkParmsFromCmdLineWinPcap: Error parsing IP address!\n"));
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }
    /* get mode */
    if (!ParseLinkMode(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.eLinkMode))
    {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "CreateLinkParmsFromCmdLineWinPcap: Error parsing LinkMode!\n"));
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }

    /* get optional parameters */
    for (*ptcWord = OsStrtok(EC_NULL, " "); *ptcWord != EC_NULL;)
    {
        if (0 == OsStricmp(*ptcWord, "--adapterid"))
        {
            if (!ParseString(ptcWord, lpCmdLine, tcStorage))
            {
                dwRetVal = EC_E_INVALIDPARM;
                goto Exit;
            }
            OsStrncpy(pLinkParmsAdapter->szAdapterId, *ptcWord, MAX_LEN_WINPCAP_ADAPTER_ID);
            pLinkParmsAdapter->szAdapterId[MAX_LEN_WINPCAP_ADAPTER_ID - 1] = '\0';
            *ptcWord = OsStrtok(EC_NULL, " ");
        }
        else
        {
            *pbGetNextWord = EC_FALSE;
            break;
        }
    }

    /* no errors */
    *ppLinkParms = &pLinkParmsAdapter->linkParms;
    dwRetVal = EC_E_NOERROR;

Exit:
    if (EC_E_NOERROR != dwRetVal)
    {
        SafeOsFree(pLinkParmsAdapter);
    }
    return dwRetVal;
}
#endif /* INCLUDE_EMLLWINPCAP */

#if (defined INCLUDE_EMLLUDP)
/***************************************************************************************************/
/**
\brief  Create UDP link layer parameters according to current command line parsing

\return EC_E_NOERROR     if link layer parameters was created
        EC_E_NOTFOUND    if command line was not matching
        EC_E_INVALIDPARM if syntax error
*/
static EC_T_DWORD CreateLinkParmsFromCmdLineUdp(EC_T_CHAR** ptcWord, EC_T_CHAR** lpCmdLine, EC_T_CHAR* tcStorage, EC_T_BOOL* pbGetNextWord,
    EC_T_LINK_PARMS** ppLinkParms)
{
    EC_T_DWORD              dwRetVal          = EC_E_ERROR;
    EC_T_LINK_PARMS_UDP*    pLinkParmsAdapter = EC_NULL;

    EC_UNREFPARM(lpCmdLine);
    EC_UNREFPARM(tcStorage);
    EC_UNREFPARM(pbGetNextWord);

    /* check for matching adapter */
    if (OsStricmp(*ptcWord, "-udp") != 0)
    {
        dwRetVal = EC_E_NOTFOUND;
        goto Exit;
    }
    /* alloc adapter specific link parms */
    pLinkParmsAdapter = (EC_T_LINK_PARMS_UDP*)OsMalloc(sizeof(EC_T_LINK_PARMS_UDP));
    if (EC_NULL == pLinkParmsAdapter)
    {
        dwRetVal = EC_E_NOMEMORY;
        goto Exit;
    }
    OsMemset(pLinkParmsAdapter, 0, sizeof(EC_T_LINK_PARMS_UDP));
    LinkParmsInit(&pLinkParmsAdapter->linkParms, EC_LINK_PARMS_SIGNATURE_UDP, sizeof(EC_T_LINK_PARMS_UDP), EC_LINK_PARMS_IDENT_UDP, 1, EcLinkMode_POLLING);

    /* parse IP address */
    *ptcWord = OsStrtok(EC_NULL, ".");
    if ((*ptcWord == EC_NULL) || (OsStrncmp(*ptcWord, "-", 1) == 0))
    {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "CreateLinkParmsFromCmdLineUdp: IP address missing!\n"));
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }
    if (!ParseIpAddress(*ptcWord, pLinkParmsAdapter->abyIpAddress))
    {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "CreateLinkParmsFromCmdLineUdp: Error parsing IP address!\n"));
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }
    if (!ParseLinkMode(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.eLinkMode))
    {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "CreateLinkParmsFromCmdLineUdp: Error parsing link mode!\n"));
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }
    /* TODO Remove?*/
    pLinkParmsAdapter->wPort = 0x88a4;

    /* if (!ParseWord(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->wPort))
    {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "CreateLinkParmsFromCmdLineUdp: Error parsing port number!\n"));
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    } */

    /* no errors */
    *ppLinkParms = &pLinkParmsAdapter->linkParms;
    dwRetVal = EC_E_NOERROR;

Exit:
    if (EC_E_NOERROR != dwRetVal)
    {
        SafeOsFree(pLinkParmsAdapter);
    }
    return dwRetVal;
}
#endif /* INCLUDE_EMLLUDP */

#if (defined INCLUDE_EMLLNDIS)
/***************************************************************************************************/
/**
\brief  Create NDIS link layer parameters according to current command line parsing

\return EC_E_NOERROR     if link layer parameters was created
EC_E_NOTFOUND    if command line was not matching
EC_E_INVALIDPARM if syntax error
*/
static EC_T_DWORD CreateLinkParmsFromCmdLineNdis(EC_T_CHAR** ptcWord, EC_T_CHAR** lpCmdLine, EC_T_CHAR* tcStorage, EC_T_BOOL* pbGetNextWord,
    EC_T_LINK_PARMS** ppLinkParms)
{
    EC_T_DWORD            dwRetVal          = EC_E_ERROR;
    EC_T_LINK_PARMS_NDIS* pLinkParmsAdapter = EC_NULL;

    EC_UNREFPARM(lpCmdLine);
    EC_UNREFPARM(tcStorage);
    EC_UNREFPARM(pbGetNextWord);

    /* check for matching adapter */
    if (OsStricmp(*ptcWord, "-ndis") != 0)
    {
        dwRetVal = EC_E_NOTFOUND;
        goto Exit;
    }
    /* alloc adapter specific link parms */
    pLinkParmsAdapter = (EC_T_LINK_PARMS_NDIS*)OsMalloc(sizeof(EC_T_LINK_PARMS_NDIS));
    if (EC_NULL == pLinkParmsAdapter)
    {
        dwRetVal = EC_E_NOMEMORY;
        goto Exit;
    }
    OsMemset(pLinkParmsAdapter, 0, sizeof(EC_T_LINK_PARMS_NDIS));
    LinkParmsInit(&pLinkParmsAdapter->linkParms, EC_LINK_PARMS_SIGNATURE_NDIS, sizeof(EC_T_LINK_PARMS_NDIS), EC_LINK_PARMS_IDENT_NDIS, 1, EcLinkMode_POLLING);

    /* parse IP address */
    *ptcWord = OsStrtok(EC_NULL, ".");
    if ((*ptcWord == EC_NULL) || (OsStrncmp(*ptcWord, "-", 1) == 0))
    {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "CreateLinkParmsFromCmdLineNdis: IP address missing!\n"));
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }
    if (!ParseIpAddress(*ptcWord, pLinkParmsAdapter->abyIpAddress))
    {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "CreateLinkParmsFromCmdLineNdis: Error parsing IP address!\n"));
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }
    if (!ParseLinkMode(ptcWord, lpCmdLine, tcStorage, &pLinkParmsAdapter->linkParms.eLinkMode))
    {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "CreateLinkParmsFromCmdLineNdis: Error parsing link mode!\n"));
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }

    /* get optional parameters */
    for (*ptcWord = OsStrtok(EC_NULL, " "); *ptcWord != EC_NULL;)
    {
        if (0 == OsStricmp(*ptcWord, "--name"))
        {
            if (!ParseString(ptcWord, lpCmdLine, tcStorage))
            {
                dwRetVal = EC_E_INVALIDPARM;
                goto Exit;
            }
            OsStrncpy(pLinkParmsAdapter->szAdapterName, *ptcWord, EC_NDIS_ADAPTER_NAME_MAXLEN);
            *ptcWord = OsStrtok(EC_NULL, " ");
        }
        else  if (0 == OsStricmp(*ptcWord, "DisablePromiscuousMode"))
        {
            pLinkParmsAdapter->bDisablePromiscuousMode = EC_TRUE;
            *ptcWord = OsStrtok(EC_NULL, " ");
        }
        else  if (0 == OsStricmp(*ptcWord, "DisableForceBroadcast"))
        {
            pLinkParmsAdapter->bDisableForceBroadcast = EC_TRUE;
            *ptcWord = OsStrtok(EC_NULL, " ");
        }
        else
        {
            *pbGetNextWord = EC_FALSE;
            break;
        }
    }

    /* no errors */
    *ppLinkParms = &pLinkParmsAdapter->linkParms;
    dwRetVal = EC_E_NOERROR;

Exit:
    if (EC_E_NOERROR != dwRetVal)
    {
        SafeOsFree(pLinkParmsAdapter);
    }
    return dwRetVal;
}
#endif /* INCLUDE_EMLLNDIS */

/***************************************************************************************************/
/**
\brief  Select Link Layer.

This function checks whether parameter portion is a LinkLayer information and processes it
\return EC_TRUE if parameter is LinkLayer Portion, EC_FALSE otherwise.
*/
EC_T_DWORD CreateLinkParmsFromCmdLine
(   EC_T_CHAR**         ptcWord,
    EC_T_CHAR**         lpCmdLine,
    EC_T_CHAR*          tcStorage,
    EC_T_BOOL*          pbGetNextWord,  /* [out]  Shows that next parameter should be read or not */
    EC_T_LINK_PARMS**   ppLinkParms,
    EC_T_LINK_TTS*      pTtsParms
)
{
EC_T_DWORD dwRetVal = EC_E_NOTFOUND;

#if (defined INCLUDE_EMLLALTERATSE)
    dwRetVal = CreateLinkParmsFromCmdLineAlteraTse(ptcWord, lpCmdLine, tcStorage, pbGetNextWord, ppLinkParms);
    if (EC_E_NOTFOUND != dwRetVal)
    {
        goto Exit;
    }
#endif
#if (defined INCLUDE_EMLL_SOC_BCMGENET)
    dwRetVal = CreateLinkParmsFromCmdLineBcmGenet(ptcWord, lpCmdLine, tcStorage, pbGetNextWord, ppLinkParms);
    if (EC_E_NOTFOUND != dwRetVal)
    {
        goto Exit;
    }
#endif
#if (defined INCLUDE_EMLLANTAIOS)
    dwRetVal = CreateLinkParmsFromCmdLineAntaios(ptcWord, lpCmdLine, tcStorage, pbGetNextWord, ppLinkParms);
    if (EC_E_NOTFOUND != dwRetVal)
    {
        goto Exit;
    }
#endif
#if (defined INCLUDE_EMLLCCAT)
    dwRetVal = CreateLinkParmsFromCmdLineCCAT(ptcWord, lpCmdLine, tcStorage, pbGetNextWord, ppLinkParms);
    if (EC_E_NOTFOUND != dwRetVal)
    {
        goto Exit;
    }
#endif
#if (defined INCLUDE_EMLLCPSW)
    dwRetVal = CreateLinkParmsFromCmdLineCPSW(ptcWord, lpCmdLine, tcStorage, pbGetNextWord, ppLinkParms);
    if (EC_E_NOTFOUND != dwRetVal)
    {
        goto Exit;
    }
#endif
#if (defined INCLUDE_EMLLDUMMY)
    dwRetVal = CreateLinkParmsFromCmdLineDummy(ptcWord, lpCmdLine, tcStorage, pbGetNextWord, ppLinkParms);
    if (EC_E_NOTFOUND != dwRetVal)
    {
        goto Exit;
    }
#endif
#if (defined INCLUDE_EMLLDW3504)
    dwRetVal = CreateLinkParmsFromCmdLineDW3504(ptcWord, lpCmdLine, tcStorage, pbGetNextWord, ppLinkParms);
    if (EC_E_NOTFOUND != dwRetVal)
    {
        goto Exit;
    }
#endif
#if (defined INCLUDE_EMLLEG20T)
    dwRetVal = CreateLinkParmsFromCmdLineEG20T(ptcWord, lpCmdLine, tcStorage, pbGetNextWord, ppLinkParms);
    if (EC_E_NOTFOUND != dwRetVal)
    {
        goto Exit;
    }
#endif
#if (defined INCLUDE_EMLLEMAC)
    dwRetVal = CreateLinkParmsFromCmdLineEMAC(ptcWord, lpCmdLine, tcStorage, pbGetNextWord, ppLinkParms);
    if (EC_E_NOTFOUND != dwRetVal)
    {
        goto Exit;
    }
#endif
#if (defined INCLUDE_EMLLETSEC)
    dwRetVal = CreateLinkParmsFromCmdLineETSEC(ptcWord, lpCmdLine, tcStorage, pbGetNextWord, ppLinkParms);
    if (EC_E_NOTFOUND != dwRetVal)
    {
        goto Exit;
    }
#endif
#if (defined INCLUDE_EMLLFSLFEC)
    dwRetVal = CreateLinkParmsFromCmdLineFslFec(ptcWord, lpCmdLine, tcStorage, pbGetNextWord, ppLinkParms);
    if (EC_E_NOTFOUND != dwRetVal)
    {
        goto Exit;
    }
#endif
#if (defined INCLUDE_EMLLGEM)
    dwRetVal = CreateLinkParmsFromCmdLineGEM(ptcWord, lpCmdLine, tcStorage, pbGetNextWord, ppLinkParms);
    if (EC_E_NOTFOUND != dwRetVal)
    {
        goto Exit;
    }
#endif
#if (defined INCLUDE_EMLLI8254X)
    dwRetVal = CreateLinkParmsFromCmdLineI8254x(ptcWord, lpCmdLine, tcStorage, pbGetNextWord, ppLinkParms);
    if (EC_E_NOTFOUND != dwRetVal)
    {
        goto Exit;
    }
#endif
#if (defined INCLUDE_EMLLI8255X)
    dwRetVal = CreateLinkParmsFromCmdLineI8255x(ptcWord, lpCmdLine, tcStorage, pbGetNextWord, ppLinkParms);
    if (EC_E_NOTFOUND != dwRetVal)
    {
        goto Exit;
    }
#endif
#if (defined INCLUDE_EMLLICSS)
    dwRetVal = CreateLinkParmsFromCmdLineICSS(ptcWord, lpCmdLine, tcStorage, pbGetNextWord, ppLinkParms, pTtsParms);
    if (EC_E_NOTFOUND != dwRetVal)
    {
        goto Exit;
    }
#else
    EC_UNREFPARM(pTtsParms);
#endif
#if (defined INCLUDE_EMLLICSSG)
    dwRetVal = CreateLinkParmsFromCmdLineICSSG(ptcWord, lpCmdLine, tcStorage, pbGetNextWord, ppLinkParms, pTtsParms);
    if (EC_E_NOTFOUND != dwRetVal)
    {
        goto Exit;
    }
#endif
#if (defined INCLUDE_EMLLL9218I)
    dwRetVal = CreateLinkParmsFromCmdLineL9218i(ptcWord, lpCmdLine, tcStorage, pbGetNextWord, ppLinkParms);
    if (EC_E_NOTFOUND != dwRetVal)
    {
        goto Exit;
    }
#endif
#if (defined INCLUDE_EMLLLAN743X)
    dwRetVal = CreateLinkParmsFromCmdLineLAN743x(ptcWord, lpCmdLine, tcStorage, pbGetNextWord, ppLinkParms);
    if (EC_E_NOTFOUND != dwRetVal)
    {
        goto Exit;
    }
#endif
#if (defined INCLUDE_EMLLNDIS)
    dwRetVal = CreateLinkParmsFromCmdLineNdis(ptcWord, lpCmdLine, tcStorage, pbGetNextWord, ppLinkParms);
    if (EC_E_NOTFOUND != dwRetVal)
    {
        goto Exit;
    }
#endif
#if (defined INCLUDE_EMLLNDISUIO)
    dwRetVal = CreateLinkParmsFromCmdLineNDISUIO(ptcWord, lpCmdLine, tcStorage, pbGetNextWord, ppLinkParms);
    if (EC_E_NOTFOUND != dwRetVal)
    {
        goto Exit;
    }
#endif
#if (defined INCLUDE_EMLLPROXY)
    dwRetVal = CreateLinkParmsFromCmdLineProxy(ptcWord, lpCmdLine, tcStorage, pbGetNextWord, ppLinkParms);
    if (EC_E_NOTFOUND != dwRetVal)
    {
        goto Exit;
    }
#endif
#if (defined INCLUDE_EMLLR6040)
    dwRetVal = CreateLinkParmsFromCmdLineR6040(ptcWord, lpCmdLine, tcStorage, pbGetNextWord, ppLinkParms);
    if (EC_E_NOTFOUND != dwRetVal)
    {
        goto Exit;
    }
#endif
#if (defined INCLUDE_EMLLRIN32)
    dwRetVal = CreateLinkParmsFromCmdLineRIN32(ptcWord, lpCmdLine, tcStorage, pbGetNextWord, ppLinkParms);
    if (EC_E_NOTFOUND != dwRetVal)
    {
        goto Exit;
    }
#endif
#if (defined INCLUDE_EMLLRTL8139)
    dwRetVal = CreateLinkParmsFromCmdLineRTL8139(ptcWord, lpCmdLine, tcStorage, pbGetNextWord, ppLinkParms);
    if (EC_E_NOTFOUND != dwRetVal)
    {
        goto Exit;
    }
#endif
#if (defined INCLUDE_EMLLRTL8169)
    dwRetVal = CreateLinkParmsFromCmdLineRTL8169(ptcWord, lpCmdLine, tcStorage, pbGetNextWord, ppLinkParms);
    if (EC_E_NOTFOUND != dwRetVal)
    {
        goto Exit;
    }
#endif
#if (defined INCLUDE_EMLLRZT1)
    dwRetVal = CreateLinkParmsFromCmdLineRZT1(ptcWord, lpCmdLine, tcStorage, pbGetNextWord, ppLinkParms);
    if (EC_E_NOTFOUND != dwRetVal)
    {
        goto Exit;
    }
#endif
#if (defined INCLUDE_EMLLSHETH)
    dwRetVal = CreateLinkParmsFromCmdLineSHEth(ptcWord, lpCmdLine, tcStorage, pbGetNextWord, ppLinkParms);
    if (EC_E_NOTFOUND != dwRetVal)
    {
        goto Exit;
    }
#endif
#if (defined INCLUDE_EMLLSIMULATOR)
    dwRetVal = CreateLinkParmsFromCmdLineSimulator(ptcWord, lpCmdLine, tcStorage, pbGetNextWord, ppLinkParms);
    if (EC_E_NOTFOUND != dwRetVal)
    {
        goto Exit;
    }
#endif
#if (defined INCLUDE_EMLLSNARF)
    dwRetVal = CreateLinkParmsFromCmdLineSnarf(ptcWord, lpCmdLine, tcStorage, pbGetNextWord, ppLinkParms);
    if (EC_E_NOTFOUND != dwRetVal)
    {
        goto Exit;
    }
#endif
#if (defined INCLUDE_EMLLSOCKRAW)
    dwRetVal = CreateLinkParmsFromCmdLineSockRaw(ptcWord, lpCmdLine, tcStorage, pbGetNextWord, ppLinkParms);
    if (EC_E_NOTFOUND != dwRetVal)
    {
        goto Exit;
    }
#endif
#if (defined INCLUDE_EMLLSTM32ETH)
    dwRetVal = CreateLinkParmsFromCmdLineStm32Eth(ptcWord, lpCmdLine, tcStorage, pbGetNextWord, ppLinkParms);
    if (EC_E_NOTFOUND != dwRetVal)
    {
        goto Exit;
    }
#endif
#if (defined INCLUDE_EMLLTIENETICSSG)
    dwRetVal = CreateLinkParmsFromCmdLineTiEnetIcssg(ptcWord, lpCmdLine, tcStorage, pbGetNextWord, ppLinkParms);
    if (EC_E_NOTFOUND != dwRetVal)
    {
        goto Exit;
    }
#endif
#if (defined INCLUDE_EMLLWINPCAP)
    dwRetVal = CreateLinkParmsFromCmdLineWinPcap(ptcWord, lpCmdLine, tcStorage, pbGetNextWord, ppLinkParms);
    if (EC_E_NOTFOUND != dwRetVal)
    {
        goto Exit;
    }
#endif
#if (defined INCLUDE_EMLLUDP)
    dwRetVal = CreateLinkParmsFromCmdLineUdp(ptcWord, lpCmdLine, tcStorage, pbGetNextWord, ppLinkParms);
    if (EC_E_NOTFOUND != dwRetVal)
    {
        goto Exit;
    }
#endif

Exit:
    return dwRetVal;
}

EC_T_VOID FreeLinkParms(EC_T_LINK_PARMS* pLinkParms)
{
#if (defined INCLUDE_EMLLSIMULATOR)
    if (EC_LINK_PARMS_SIGNATURE_SIMULATOR == pLinkParms->dwSignature)
    {
        EC_T_LINK_PARMS_SIMULATOR* pLinkParmsSimulator = (EC_T_LINK_PARMS_SIMULATOR*)pLinkParms;
        EC_T_DWORD dwIdx2 = 0;

        SafeOsFree(pLinkParmsSimulator->pbyCnfData);

        for (dwIdx2 = 0; dwIdx2 < EC_SIMULATOR_MAX_LINK_PARMS; dwIdx2++)
        {
            SafeOsFree(pLinkParmsSimulator->apLinkParms[dwIdx2]);
        }
    }
#endif
    SafeOsFree(pLinkParms);
}

EC_T_VOID ShowSyntaxLinkLayer(EC_T_VOID)
{
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "Link layer options:\n"));
#if (defined INCLUDE_EMLLALTERATSE)
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -alteratse        Link layer = Lenze/Intel FPGA TSE\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Instance        Port Instance (1=first), e.g. 1\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Mode            Interrupt (0) or Polling (1) mode\n"));
#endif
#if (defined INCLUDE_EMLLANTAIOS)
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -antaios          Link layer = Antaios link layer\n"));
#endif
#if (defined INCLUDE_EMLLCCAT)
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -ccat             Link layer = Beckhoff CCAT\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Instance        Device instance (1=first), e.g. 1\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Mode            Interrupt (0) or Polling (1) mode\n"));
#endif
#if (defined INCLUDE_EMLL_SOC_BCMGENET)
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -bcmgenet         Link layer = Broadcom BcmGenet\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Instance        Device instance (1=first), e.g. 1\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Mode            Interrupt (0) or Polling (1) mode\n"));
#endif
#if (defined INCLUDE_EMLLCPSW)
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -cpsw             Link layer = Texas Instruments Common Platform Switch (CPSW)\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Instance        Device instance (port) 1 P1, 2 P2\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Mode            Interrupt (0) or Polling (1) mode\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     PortPriority    Low priority (0) or high priority (1)\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     MasterFlag      (m) Master (Initialize Switch), (s) Slave\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "    [RefBoard:       bone | am3359-icev2 | am437x-idk | am572x-idk | 387X_evm | custom\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "      if custom       CpswType: am33XX | am437X | am57X | am387X\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "      if custom       PhyAddress: 0 ... 31\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "      if custom       PhyInterface: rmii | gmii | rgmii \n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "      if custom       NotUseDmaBuffers: FALSE (0) or TRUE (1)]\n"));
#endif
#if (defined INCLUDE_EMLLDUMMY)
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -dummy            Link layer = Dummy\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Instance        Device instance (1=first), e.g. 1\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Mode            Interrupt (0) or Polling (1) mode\n"));
#endif
#if (defined INCLUDE_EMLLDW3504)
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -dw3504           Link layer = Synopsys DesignWare 3504-0 Universal 10/100/1000 Ethernet MAC (DW3504)\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Instance        Device instance 1 for emac0, 2 for emac1\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Mode            Interrupt (0) or Polling (1) mode\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "    [RefBoard:       intel_atom | lces1 | rd55up06 | r12ccpu | rzn1 | socrates | stm32mp157a-dk1 | custom\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     if custom        DW3504Type: intel_atom | cycloneV | lces1 | stm32mp157a-dk1 \n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     if custom        PhyInterface: fixed | mii | rmii | gmii | sgmii | rgmii | osdriver \n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     if custom        PhyAddress: 0 ... 31, default 0]\n"));
#endif
#if (defined INCLUDE_EMLLEG20T)
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -eg20t            Link layer = EG20T Gigabit Ethernet Controller\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Instance        Device instance (1=first), e.g. 1\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Mode            Interrupt (0) or Polling (1) mode\n"));
#endif
#if (defined INCLUDE_EMLLEMAC)
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -emac             Link layer = Xilinx LogiCORE IP XPS EMAC\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Instance        Device instance (must be 1)\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Mode            Interrupt (0) or Polling (1) mode\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "    [RefBoard:       MC2002E | custom\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     if custom        RegisterBase: register base address (hex value)\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     if custom        RegisterLength: register length (hex value)\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     if custom        NotUseDmaBuffers: FALSE (0) or TRUE (1)]\n"));
#endif
#if (defined INCLUDE_EMLLETSEC)
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -fsletsec         Link layer = Freescale TSEC / eTSEC V1 / eTSEC V2 (VeTSEC)\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Instance        Device instance (1=first), e.g. 1\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Mode            Interrupt (0) or Polling (1) mode\n"));
#if (defined EC_VERSION_VXWORKS)
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "    RefBoard:        p2020rdb | twrp1025 | istmpc8548 | xj_epu20c | twrls1021a | tqmls_ls102xa | custom\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     if custom        ETSEC type: p2020rdb | twrp1025 | istmpc8548 | xj_epu20c | twrls1021a | tqmls_ls102xa\n"));
#else
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "    RefBoard:        p2020rdb | twrp1025 | twrls1021a | tqmls_ls102xa | custom\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     if custom        ETSEC type: p2020rdb | twrp1025 | twrls1021a | tqmls_ls102xa\n"));
#endif
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     if custom        PhyAddress: 0 ... 31, default 0\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     if custom        RX IRQ: Default depending on ETSEC type\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     if custom        NotUseDmaBuffers: FALSE (0) or TRUE (1), default 0\n"));
#endif
#if (defined INCLUDE_EMLLFSLFEC)
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -fslfec           Link layer = Freescale FEC\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Instance        Device instance (1=first), e.g. 1\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Mode            Interrupt (0) or Polling (1) mode\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "    [RefBoard:       mars | sabrelite | sabresd | imx28evk | topaz | imxceetul2 | mimxrt1064-evk | custom\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     if custom        FecType: imx25 | imx28 | imx53 | imx6 | vf6 | imx7 | imx8 | imx8m | imxrt1064\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     if custom        PhyInterface: fixed | mii | rmii | rmii50Mhz | gmii | sgmii | rgmii | osdriver\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     if custom        PhyAddress: 0 ... 31, default 0 (don't use if osdriver)]\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "    [nopinmuxing     no pin muxing]\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "    [nomacaddr       don't read MAC address]\n"));
#endif
#if (defined INCLUDE_EMLLGEM)
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -gem              Link layer = Xilinx Zynq-7000/Ultrascale (GEM)\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Instance        Device instance for GEM, GEM0 == 1...\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Mode            Interrupt (0) or Polling (1) mode\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "    [RefBoard:       zc702 | zedboard | microzed | zcu102 | zcu104 | KR260 | custom\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     if custom        PhyAddress: 0 ... 31\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     if custom        PhyConnectionMode: MIO (0) or EMIO (1)\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     if custom        UseGmiiToRgmii: Use Xilinx GmiiToRgmii converter TRUE (1) or FALSE (0)\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     if custom        GmiiToRgmiiPort: GmiiToRgmii converter PHY address 0 ... 31\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     if custom        GEM type: zynq7000 or ultrascale]\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "    [osdriver         PHY interface]\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "    [ClkDivType_K26   Clock Divisor]\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "    [nopinmuxing      don't use pin muxing]\n"));
#endif
#if (defined INCLUDE_EMLLI8254X)
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -i8254x           Link layer = Intel 8254x\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Instance        Device instance (1=first), e.g. 1\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Mode            Interrupt (0) or Polling (1) mode\n"));
#endif
#if (defined INCLUDE_EMLLI8255X)
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -i8255x           Link layer = Intel 8255x\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Instance        Device instance (1=first), e.g. 1\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Mode            Interrupt (0) or Polling (1) mode\n"));
#endif
#if (defined INCLUDE_EMLLICSS)
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -icss             Link layer = Texas Instruments PRUICSS\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Instance        ICSS Port (100 Mbit/s) 1 ... 4\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Mode            Interrupt (0) or Polling (1) mode\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     MasterFlag      (m) Master (Initialize whole PRUSS) or (s) Slave\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     RefBoard        am572x-idk | am571x-idk | am3359-icev2 | am574x\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "    [PHY interface   mii | osdriver\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     PHY address     0 ... 31 (only for mii)]\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "    [NoPhyReset]     \n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "    [TtsEnable       tts\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     TtsCycleTime    TTS cycle time (usec)\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     TtsSendOffset   Cyclic frame send offset from cycle start (usec)]\n"));
#endif
#if (defined INCLUDE_EMLLICSSG)
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -icssg             Link layer = Texas Instruments PRUICSS\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Instance        ICSS Port (100 Mbit/s) 1 ... 4\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Mode            Interrupt (0) or Polling (1) mode\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     MasterFlag      (m) Master (Initialize whole PRUSS) or (s) Slave\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     RefBoard        am654x-idk\n"));
#endif
#if (defined INCLUDE_EMLLL9218I)
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -l9218i           Link layer = SMSC LAN9218i/LAN9221\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Mode            Interrupt (0) or Polling (1) mode\n"));
#endif
#if (defined INCLUDE_EMLLLAN743X)
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -lan743x          Link layer = Microchip LAN743xx \n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Instance        Device instance (1=first), e.g. 1\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Mode            Interrupt (0) or Polling (1) mode\n"));
#endif
#if (defined INCLUDE_EMLLNDIS)
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -ndis                         Link layer = NDIS\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     IpAddress                   IP address of network adapter card, e.g. 192.168.157.2 or 0.0.0.0 if name given (optional)\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Mode                        Interrupt (0) or Polling (1) mode\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "    [--name                      adapter name\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     ServiceName                 service name from HKEY_LOCAL_MACHINE\\SOFTWARE\\Microsoft\\Windows NT\\CurrentVersion\\NetworkCards]\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "    [DisablePromiscuousMode      \n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "    [DisableForceBroadcast       \n"));
#endif
#if (defined INCLUDE_EMLLNDISUIO)
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -ndisuio          Link layer = NdisUio\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Adapter         Device name (registry), e.g. PCI\\RTL81391\n"));
#endif
#if (defined INCLUDE_EMLLPROXY) && 0
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -proxy            Link layer = Proxy\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Instance        Device instance (1=first), e.g. 1\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Mode            Interrupt (0) or Polling (1) mode\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Src IP          Source adapter IP address (listen)\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Src port        Source port number (listen)\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Dst IP          Destination adapter IP address (listen)\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Dst port        Destination port number (listen)\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "    [--mac           MAC address, formatted as xx:xx:xx:xx:xx:xx or xx-xx-xx-xx-xx-xx]\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "    [--rxbuffercnt   Frame buffer count for interrupt service thread (IST)]\n"));
#endif
#if (defined INCLUDE_EMLLR6040)
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -r6040            Link layer = R6040\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Instance        Device instance (1=first), e.g. 1\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Mode            Interrupt (0) or Polling (1) mode\n"));
#endif
#if (defined INCLUDE_EMLLRIN32)
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -rin32            Link layer = RIN32\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Instance        Device instance (1=first), e.g. 1\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Mode            Interrupt (0) or Polling (1) mode\n"));
#endif
#if (defined INCLUDE_EMLLRTL8139)
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -rtl8139          Link layer = Realtek RTL8139\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Instance        Device instance (1=first), e.g. 1\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Mode            Interrupt (0) or Polling (1) mode\n"));
#endif
#if (defined INCLUDE_EMLLRTL8169)
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -rtl8169          Link layer = Realtek RTL8169 / RTL8168 / RTL8111\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Instance        Device instance (1=first), e.g. 1\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Mode            Interrupt (0) or Polling (1) mode\n"));
#endif
#if (defined INCLUDE_EMLLRZT1)
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -rzt1             Link layer = RZT1\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Instance        Device instance 1 for Port 0 or 2 for Port 1\n"));
#endif
#if (defined INCLUDE_EMLLSHETH)
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -sheth            Link layer = Super H Ethernet controller\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Instance        Device instance (1=first), e.g. 1\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Mode            Interrupt (0) or Polling (1) mode\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     RefBoard        a800eva | rzg1e\n"));
#endif
#if (defined INCLUDE_EMLLSIMULATOR)
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -simulator        Link layer = EC-Simulator\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Instance        Device instance (1=first), e.g. 1\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Mode            Interrupt (0) or Polling (1) mode\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     EXI-file        simulated topology\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "    [--mac           set MAC address\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "       address       MAC address, e.g. 00-05-94-03-3C-9C or 00:05:94:03:3C:9C]\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "    [--lic           Use License key\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "       key           license key]\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "    [--link          link parms, e.g. used for license check\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "       link parms    link parms, e.g. -i8254x ... ]\n"));
#if (defined INCLUDE_RAS_SERVER)
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "    [--sp            Start RAS server\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "      [port          server port]]\n"));
#endif
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "    [--connect       Connect this link layer to a specified slave port, e.g. --connect slave 1 1001 2\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "       Type          Connection type, e.g. \"slave\"\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "       Instance      Instance ID of other simulator (link layer) hosting the port to connect to, e.g. 1\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "       Address       Address of device hosting the port to connect, e.g. 1001\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "       Port          Port to connect to (0=Port A, 1=Port B, 2=Port C, 3=Port D), e.g. 2]\n"));
#endif /* INCLUDE_EMLLSIMULATOR */
#if (defined INCLUDE_EMLLSNARF)
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -snarf            Link layer = SNARF link layer\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "    AdapterName      Adapter name, e.g. fei0\n"));
#endif
#if (defined INCLUDE_EMLLSOCKRAW)
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -sockraw          Link layer = raw socket\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Device          network device (e.g. eth1)\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "    [Mode            Interrupt (0) or Polling (1) mode]\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "    [--nommaprx      disable PACKET_MMAP for receive]\n"));    
#endif
#if (defined INCLUDE_EMLLSTM32ETH)
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -stm32eth         Link layer = STM32H7 Ethernet\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Instance        Device instance (1=first), e.g. 1\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Mode            Interrupt (0) or Polling (1) mode\n"));
#endif
#if (defined INCLUDE_EMLLTIENETICSSG)
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -TiEnetIcssg   Link layer = Ti Enet LLD Icssg\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Instance        Device instance (1=first), e.g. 1\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Mode            Interrupt (0) or Polling (1) mode\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     MasterFlag      (m) Master (Initialize whole) or (s) Slave\n"));
#endif
#if (defined INCLUDE_EMLLUDP)
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -udp              Link layer = UDP\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     IpAddress       IP address of network adapter card, e.g. 192.168.157.2\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "                     NPF only: 255.255.255.x, x = network adapter number (1,2,...)\n"));
#endif
#if (defined INCLUDE_EMLLWINPCAP)
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "   -winpcap          Link layer = WinPcap/NPF\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     IpAddress       IP address of network adapter card, e.g. 192.168.157.2\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "                     NPF only: 255.255.255.x, x = network adapter number (1,2,...)\n"));
    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "     Mode            Interrupt (0) or Polling (1) mode\n"));
#endif
}

#if (defined INCLUDE_EMLL_STATIC_LIBRARY)
#include "EcLink.h"

EC_PF_LLREGISTER DemoGetLinkLayerRegFunc(EC_T_CHAR* szDriverIdent)
{
EC_PF_LLREGISTER pfLlRegister = EC_NULL;

#if (defined INCLUDE_EMLLANTAIOS)
    if (0 == OsStrcmp(EC_LINK_PARMS_IDENT_ANTAIOS, szDriverIdent))
    {
        pfLlRegister = (EC_PF_LLREGISTER)emllRegisterAntaios;
    } else
#endif
#if (defined INCLUDE_EMLL_SOC_BCMGENET)
    if (0 == OsStrcmp(EC_LINK_PARMS_IDENT_BCMGENET, szDriverIdent))
    {
        pfLlRegister = (EC_PF_LLREGISTER)emllRegisterBcmGenet;
    } else
#endif
#if (defined INCLUDE_EMLLCCAT)
    if (0 == OsStrcmp(EC_LINK_PARMS_IDENT_CCAT, szDriverIdent))
    {
        pfLlRegister = (EC_PF_LLREGISTER)emllRegisterCCAT;
    } else
#endif
#if (defined INCLUDE_EMLLCPSW)
    if (0 == OsStrcmp(EC_LINK_PARMS_IDENT_CPSW, szDriverIdent))
    {
        pfLlRegister = (EC_PF_LLREGISTER)emllRegisterCPSW;
    } else
#endif
#if (defined INCLUDE_EMLLDW3504)
    if (0 == OsStrcmp(EC_LINK_PARMS_IDENT_DW3504, szDriverIdent))
    {
        pfLlRegister = (EC_PF_LLREGISTER)emllRegisterDW3504;
    } else
#endif
#if (defined INCLUDE_EMLLETSEC)
    if (0 == OsStrcmp(EC_LINK_PARMS_IDENT_ETSEC, szDriverIdent))
    {
        pfLlRegister = (EC_PF_LLREGISTER)emllRegisterETSEC;
    } else
#endif
#if (defined INCLUDE_EMLLFSLFEC)
    if (0 == OsStrcmp(EC_LINK_PARMS_IDENT_FSLFEC, szDriverIdent))
    {
        pfLlRegister = (EC_PF_LLREGISTER)emllRegisterFslFec;
    } else
#endif
#if (defined INCLUDE_EMLLGEM)
    if (0 == OsStrcmp(EC_LINK_PARMS_IDENT_GEM, szDriverIdent))
    {
        pfLlRegister = (EC_PF_LLREGISTER)emllRegisterGEM;
    } else
#endif
#if (defined INCLUDE_EMLLI8254X)
    if (0 == OsStrcmp(EC_LINK_PARMS_IDENT_I8254X, szDriverIdent))
    {
        pfLlRegister = (EC_PF_LLREGISTER)emllRegisterI8254x;
    } else
#endif
#if (defined INCLUDE_EMLLI8255X)
    if (0 == OsStrcmp(EC_LINK_PARMS_IDENT_I8255X, szDriverIdent))
    {
        pfLlRegister = (EC_PF_LLREGISTER)emllRegisterI8255x;
    } else
#endif
#if (defined INCLUDE_EMLLICSS)
    if (0 == OsStrcmp(EC_LINK_PARMS_IDENT_ICSS, szDriverIdent))
    {
        pfLlRegister = (EC_PF_LLREGISTER)emllRegisterICSS;
    } else
#endif
#if (defined INCLUDE_EMLLICSSG)
    if (0 == OsStrcmp(EC_LINK_PARMS_IDENT_ICSSG, szDriverIdent))
    {
        pfLlRegister = (EC_PF_LLREGISTER)emllRegisterICSSG;
    } else
#endif
#if (defined INCLUDE_EMLLLAN743X)
    if (0 == OsStrcmp(EC_LINK_PARMS_IDENT_LAN743X, szDriverIdent))
    {
        pfLlRegister = (EC_PF_LLREGISTER)emllRegisterLAN743x;
    } else
#endif
#if (defined INCLUDE_EMLLPROXY)
    if (0 == OsStrcmp(EC_LINK_PARMS_IDENT_PROXY, szDriverIdent))
    {
        pfLlRegister = (EC_PF_LLREGISTER)emllRegisterProxy;
    } else
#endif
#if (defined INCLUDE_EMLLRTL8139)
    if (0 == OsStrcmp(EC_LINK_PARMS_IDENT_RTL8139, szDriverIdent))
    {
        pfLlRegister = (EC_PF_LLREGISTER)emllRegisterRTL8139;
    } else
#endif
#if (defined INCLUDE_EMLLRTL8169)
    if (0 == OsStrcmp(EC_LINK_PARMS_IDENT_RTL8169, szDriverIdent))
    {
        pfLlRegister = (EC_PF_LLREGISTER)emllRegisterRTL8169;
    } else
#endif
#if (defined INCLUDE_EMLLSIMULATOR)
    if (0 == OsStrcmp(EC_LINK_PARMS_IDENT_SIMULATOR, szDriverIdent))
    {
        pfLlRegister = (EC_PF_LLREGISTER)emllRegisterSimulator;
    }
    else
#endif
#if (defined INCLUDE_EMLLSOCKRAW)
    if (0 == OsStrcmp(EC_LINK_PARMS_IDENT_SOCKRAW, szDriverIdent))
    {
        pfLlRegister = (EC_PF_LLREGISTER)emllRegisterSockRaw;
    }
    else
#endif
#if (defined INCLUDE_EMLLSTM32ETH)
    if (0 == OsStrcmp(EC_LINK_PARMS_IDENT_STM32ETH, szDriverIdent))
    {
        pfLlRegister = (EC_PF_LLREGISTER)emllRegisterStm32Eth;
    }
    else
#endif
    {
        pfLlRegister = EC_NULL;
    }
    return pfLlRegister;
}
#endif /* INCLUDE_EMLL_STATIC_LIBRARY */

/*-END OF SOURCE FILE--------------------------------------------------------*/
