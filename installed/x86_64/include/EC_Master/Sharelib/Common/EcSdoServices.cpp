/*-----------------------------------------------------------------------------
* EcSdoServices.cpp
* Copyright                acontis technologies GmbH, Ravensburg, Germany
* Response                 Holger Oelhaf
* Description
*---------------------------------------------------------------------------*/

/*-INCLUDES------------------------------------------------------------------*/
#include "EcDemoApp.h"
#include "EcSdoServices.h"
#include "EcObjDef.h"

/*-DEFINES-------------------------------------------------------------------*/
#define LOG_BUFFER_SIZE         ((EC_T_DWORD)0x1000)

/*-FUNCTION-DEFINITIONS------------------------------------------------------*/
EC_T_VOID FlushLogBuffer(T_EC_DEMO_APP_CONTEXT* pAppContext, EC_T_CHAR* szLogBuffer)
{
    EC_UNREFPARM(pAppContext);
    if (OsStrlen(szLogBuffer) == 0) return;
    OsSnprintf(&szLogBuffer[OsStrlen(szLogBuffer)], (EC_T_INT)(LOG_BUFFER_SIZE - OsStrlen(szLogBuffer) - 1), "\n");
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "%s\n", szLogBuffer));
    szLogBuffer[0] = '\0';
}

/********************************************************************************/
/** \brief  Wait for mailbox transfer completion, log error
 *
 * \return N/A
 */
EC_T_VOID HandleMbxTferReqError(
    T_EC_DEMO_APP_CONTEXT* pAppContext,
    const EC_T_CHAR*       szErrMsg,          /**< [in] error message */
    EC_T_DWORD             dwErrorCode,       /**< [in] basic error code */
    EC_T_MBXTFER*          pMbxTfer           /**< [in] mbx transfer object */
)
{
    /* wait for MbxTfer completion, but let application finish if master never returns MbxTfer object (may happen using RAS) */
    EC_T_DWORD dwWorstCaseTimeout = 10;

    /*
     * Wait if MbxTfer still running and MbxTfer object owned by master.
     * Cannot re-use MbxTfer object while state is eMbxTferStatus_Pend.
     */
    for (dwWorstCaseTimeout = 10; (eMbxTferStatus_Pend == pMbxTfer->eTferStatus) && (dwWorstCaseTimeout > 0); dwWorstCaseTimeout--)
    {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "%s: waiting for mailbox transfer response\n", szErrMsg));
        OsSleep(2000);
    }

    if (eMbxTferStatus_Pend == pMbxTfer->eTferStatus)
    {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "%s: timeout waiting for mailbox transfer response\n", szErrMsg));
        goto Exit;
    }

    if (EC_E_NOERROR != dwErrorCode)
    {
        EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "%s: MbxTferReqError: %s (0x%lx)\n", szErrMsg, ecatGetText(dwErrorCode), dwErrorCode));
    }

Exit:
    return;
}

#if (defined INCLUDE_MASTER_OBD)
/***************************************************************************************************/
/**
 * \brief  Parse DIAG Message.
 */
EC_T_VOID ParseDiagMsg(T_EC_DEMO_APP_CONTEXT* pAppContext, EC_T_OBJ10F3_DIAGMSG* pDiag)
{
    static EC_T_CHAR szOutPut[0x200];
    const EC_T_CHAR* pszFormat       = EC_NULL;
    EC_T_CHAR*       pszWork         = EC_NULL;
    EC_T_DWORD       dwParse         = 0;
    EC_T_DWORD       dwParseLimit    = 0;
    EC_T_BYTE*       pbyParamPtr     = EC_NULL;
    EC_T_WORD        wParmFlags      = 0;
    EC_T_WORD        wParmSize       = 0;
    const EC_T_CHAR* pszSeverity     = EC_NULL;

    if (EC_NULL == pDiag)
    {
        goto Exit;
    }

    OsMemset(szOutPut, 0, sizeof(szOutPut));

    pszFormat = ecatGetText((EC_T_DWORD)pDiag->wTextId);

    if( EC_NULL == pszFormat )
    {
        goto Exit;
    }

    switch( pDiag->wFlags & 0x0F)
    {
    case DIAGFLAGINFO:  pszSeverity  = "INFO"; break;
    case DIAGFLAGWARN:  pszSeverity  = "WARN"; break;
    case DIAGFLAGERROR: pszSeverity  = " ERR"; break;
    default:            pszSeverity  = " UNK"; break;
    }

    dwParseLimit = (EC_T_DWORD) OsStrlen(pszFormat);
    pszWork = szOutPut;

    pbyParamPtr = (EC_T_BYTE*)&pDiag->oParameter;

    for( dwParse = 0; dwParse < dwParseLimit; )
    {
        switch(pszFormat[0])
        {
        case '%':
            {
                /* param */
                pszFormat++;
                dwParse++;
                if( pszFormat[0] == 'l' || pszFormat[0] == 'L' )
                {
                    pszFormat++;
                    dwParse++;
                }

                switch( pszFormat[0] )
                {
                case '%':
                    {
                        pszWork[0] = pszFormat[0];
                        pszWork++;
                        pszFormat++;
                        dwParse++;
                    } break;
                case 's':
                case 'S':
                    {
                        wParmFlags = EC_GETWORD(pbyParamPtr);

                        switch( (wParmFlags)&(0xF<<12) )
                        {
                        case DIAGPARMTYPEASCIISTRG:
                        case DIAGPARMTYPEBYTEARRAY:
                            {
                                wParmSize = (EC_T_WORD)(wParmFlags&0xFFF);
                                pbyParamPtr += sizeof(EC_T_WORD);

                                OsMemcpy(pszWork, pbyParamPtr, (wParmSize*sizeof(EC_T_BYTE)));
                                pszWork += (wParmSize*sizeof(EC_T_BYTE));
                                pbyParamPtr += wParmSize;
                                pszFormat++;
                                dwParse++;
                            } break;
                        case DIAGPARMTYPEUNICODESTRG:
                            {
                                wParmSize = (EC_T_WORD)(wParmFlags&0xFFF);
                                pbyParamPtr += sizeof(EC_T_WORD);

                                OsMemcpy(pszWork, pbyParamPtr, (wParmSize*sizeof(EC_T_WORD)));
                                pszWork += (wParmSize*sizeof(EC_T_WORD));
                                pbyParamPtr += wParmSize;
                                pszFormat++;
                                dwParse++;
                            } break;
                        case DIAGPARMTYPETEXTID:
                            {
                                const EC_T_CHAR* pszTextFromId = EC_NULL;

                                pbyParamPtr += sizeof(EC_T_WORD);
                                pszTextFromId = ecatGetText((EC_T_DWORD)EC_GETWORD(pbyParamPtr));
                                if( EC_NULL == pszTextFromId )
                                {
                                    pszWork[0] = '%';
                                    pszWork[1] = pszFormat[0];
                                    pszWork +=2;
                                    pszFormat++;
                                    dwParse++;
                                    break;
                                }
                                OsMemcpy(pszWork, pszTextFromId, OsStrlen(pszTextFromId));
                                pszWork += OsStrlen(pszTextFromId);
                                pbyParamPtr += sizeof(EC_T_WORD);
                                pszFormat++;
                                dwParse++;
                            } break;
                        default:
                            {
                                pszWork[0] = '%';
                                pszWork[1] = pszFormat[0];
                                pszWork +=2;
                                pszFormat++;
                                dwParse++;
                            } break;
                        }
                    } break;
                case 'd':
                    {
                        wParmFlags = EC_GETWORD(pbyParamPtr);

                        switch( (wParmFlags)&(0xF<<12) )
                        {
                        case DIAGPARMTYPEDATATYPE:
                            {
                                switch(wParmFlags&0xFFF)
                                {
                                case DEFTYPE_INTEGER8:
                                case DEFTYPE_UNSIGNED8:
                                    {
                                        wParmSize = sizeof(EC_T_BYTE);
                                        pbyParamPtr += sizeof(EC_T_WORD);
                                        OsSnprintf(pszWork, 4, "%d", pbyParamPtr[0]);
                                        pszWork += OsStrlen(pszWork);
                                        pbyParamPtr += wParmSize;
                                        pszFormat++;
                                        dwParse++;
                                    } break;
                                case DEFTYPE_INTEGER16:
                                case DEFTYPE_UNSIGNED16:
                                    {
                                        wParmSize = sizeof(EC_T_WORD);
                                        pbyParamPtr += sizeof(EC_T_WORD);
                                        OsSnprintf(pszWork, 7, "%d", EC_GETWORD(pbyParamPtr));
                                        pszWork += OsStrlen(pszWork);
                                        pbyParamPtr += wParmSize;
                                        pszFormat++;
                                        dwParse++;
                                    } break;
                                case DEFTYPE_BOOLEAN:
                                case DEFTYPE_INTEGER32:
                                case DEFTYPE_UNSIGNED32:
                                    {
                                        wParmSize = sizeof(EC_T_DWORD);
                                        pbyParamPtr += sizeof(EC_T_WORD);
                                        OsSnprintf(pszWork, 11, "%ld", EC_GETDWORD(pbyParamPtr));
                                        pszWork += OsStrlen(pszWork);
                                        pbyParamPtr += wParmSize;
                                        pszFormat++;
                                        dwParse++;
                                    } break;
                                case DEFTYPE_UNSIGNED24:
                                    {
                                        wParmSize = (3*sizeof(EC_T_BYTE));
                                        pbyParamPtr += sizeof(EC_T_WORD);
                                        OsSnprintf(pszWork, 11, "%ld", EC_GETDWORD(pbyParamPtr));
                                        pszWork += OsStrlen(pszWork);
                                        pbyParamPtr += wParmSize;
                                        pszFormat++;
                                        dwParse++;
                                    } break;
                                default:
                                    {
                                        pszWork[0] = '%';
                                        pszWork[1] = pszFormat[0];
                                        pszWork +=2;
                                        pszFormat++;
                                        dwParse++;
                                    };
                                }
                            } break;
                        default:
                            {
                                pszWork[0] = '%';
                                pszWork[1] = pszFormat[0];
                                pszWork +=2;
                                pszFormat++;
                                dwParse++;
                            } break;
                        }
                    } break;
                case 'x':
                    {
                        wParmFlags = EC_GETWORD(pbyParamPtr);

                        switch( (wParmFlags)&(0xF<<12) )
                        {
                        case DIAGPARMTYPEDATATYPE:
                            {
                                switch(wParmFlags&0xFFF)
                                {
                                case DEFTYPE_INTEGER8:
                                case DEFTYPE_UNSIGNED8:
                                    {
                                        wParmSize = sizeof(EC_T_BYTE);
                                        pbyParamPtr += sizeof(EC_T_WORD);
                                        OsSnprintf(pszWork, 3, "%x", pbyParamPtr[0]);
                                        pszWork += OsStrlen(pszWork);
                                        pbyParamPtr += wParmSize;
                                        pszFormat++;
                                        dwParse++;
                                    } break;
                                case DEFTYPE_INTEGER16:
                                case DEFTYPE_UNSIGNED16:
                                    {
                                        wParmSize = sizeof(EC_T_WORD);
                                        pbyParamPtr += sizeof(EC_T_WORD);
                                        OsSnprintf(pszWork, 5, "%x", EC_GETWORD(pbyParamPtr));
                                        pszWork += OsStrlen(pszWork);
                                        pbyParamPtr += wParmSize;
                                        pszFormat++;
                                        dwParse++;
                                    } break;
                                case DEFTYPE_BOOLEAN:
                                case DEFTYPE_INTEGER32:
                                case DEFTYPE_UNSIGNED32:
                                    {
                                        wParmSize = sizeof(EC_T_DWORD);
                                        pbyParamPtr += sizeof(EC_T_WORD);
                                        OsSnprintf(pszWork, 9, "%lx", EC_GETDWORD(pbyParamPtr));
                                        pszWork += OsStrlen(pszWork);
                                        pbyParamPtr += wParmSize;
                                        pszFormat++;
                                        dwParse++;
                                    } break;
                                case DEFTYPE_UNSIGNED24:
                                    {
                                        wParmSize = (3*sizeof(EC_T_BYTE));
                                        pbyParamPtr += sizeof(EC_T_WORD);
                                        OsSnprintf(pszWork, 7, "%lx", EC_GETDWORD(pbyParamPtr));
                                        pszWork += OsStrlen(pszWork);
                                        pbyParamPtr += wParmSize;
                                        pszFormat++;
                                        dwParse++;
                                    } break;
                                default:
                                    {
                                        pszWork[0] = '%';
                                        pszWork[1] = pszFormat[0];
                                        pszWork +=2;
                                        pszFormat++;
                                        dwParse++;
                                    };
                                }
                            } break;
                        default:
                            {
                                pszWork[0] = '%';
                                pszWork[1] = pszFormat[0];
                                pszWork +=2;
                                pszFormat++;
                                dwParse++;
                            } break;
                        }
                    } break;
                case 'X':
                    {
                        wParmFlags = EC_GETWORD(pbyParamPtr);

                        switch( (wParmFlags)&(0xF<<12) )
                        {
                        case DIAGPARMTYPEDATATYPE:
                            {
                                switch(wParmFlags&0xFFF)
                                {
                                case DEFTYPE_INTEGER8:
                                case DEFTYPE_UNSIGNED8:
                                    {
                                        wParmSize = sizeof(EC_T_BYTE);
                                        pbyParamPtr += sizeof(EC_T_WORD);
                                        OsSnprintf(pszWork, 3, "%X", pbyParamPtr[0]);
                                        pszWork += OsStrlen(pszWork);
                                        pbyParamPtr += wParmSize;
                                        pszFormat++;
                                        dwParse++;
                                    } break;
                                case DEFTYPE_INTEGER16:
                                case DEFTYPE_UNSIGNED16:
                                    {
                                        wParmSize = sizeof(EC_T_WORD);
                                        pbyParamPtr += sizeof(EC_T_WORD);
                                        OsSnprintf(pszWork, 5, "%X", EC_GETWORD(pbyParamPtr));
                                        pszWork += OsStrlen(pszWork);
                                        pbyParamPtr += wParmSize;
                                        pszFormat++;
                                        dwParse++;
                                    } break;
                                case DEFTYPE_BOOLEAN:
                                case DEFTYPE_INTEGER32:
                                case DEFTYPE_UNSIGNED32:
                                    {
                                        wParmSize = sizeof(EC_T_DWORD);
                                        pbyParamPtr += sizeof(EC_T_WORD);
                                        OsSnprintf(pszWork, 9, "%lX", EC_GETDWORD(pbyParamPtr));
                                        pszWork += OsStrlen(pszWork);
                                        pbyParamPtr += wParmSize;
                                        pszFormat++;
                                        dwParse++;
                                    } break;
                                case DEFTYPE_UNSIGNED24:
                                    {
                                        wParmSize = (3*sizeof(EC_T_BYTE));
                                        pbyParamPtr += sizeof(EC_T_WORD);
                                        OsSnprintf(pszWork, 7, "%lX", EC_GETDWORD(pbyParamPtr));
                                        pszWork += OsStrlen(pszWork);
                                        pbyParamPtr += wParmSize;
                                        pszFormat++;
                                        dwParse++;
                                    } break;
                                default:
                                    {
                                        pszWork[0] = '%';
                                        pszWork[1] = pszFormat[0];
                                        pszWork +=2;
                                        pszFormat++;
                                        dwParse++;
                                    };
                                }
                            } break;
                        default:
                            {
                                pszWork[0] = '%';
                                pszWork[1] = pszFormat[0];
                                pszWork +=2;
                                pszFormat++;
                                dwParse++;
                            } break;
                        }
                    } break;
                case 'C':
                case 'c':
                    {
                        wParmFlags = EC_GETWORD(pbyParamPtr);

                        switch( (wParmFlags)&(0xF<<12) )
                        {
                        case DIAGPARMTYPEDATATYPE:
                            {
                                switch(wParmFlags&0xFFF)
                                {
                                case DEFTYPE_INTEGER8:
                                case DEFTYPE_UNSIGNED8:
                                    {
                                        wParmSize = sizeof(EC_T_BYTE);
                                        pbyParamPtr += sizeof(EC_T_WORD);
                                        pszWork[0] = pbyParamPtr[0];
                                        pszWork ++;
                                        pbyParamPtr ++;
                                        pszFormat ++;
                                        dwParse++;
                                    } break;
                                default:
                                    {
                                        pszWork[0] = '%';
                                        pszWork[1] = pszFormat[0];
                                        pszWork +=2;
                                        pszFormat++;
                                        dwParse++;
                                    };
                                }
                            } break;
                        default:
                            {
                                pszWork[0] = '%';
                                pszWork[1] = pszFormat[0];
                                pszWork +=2;
                                pszFormat++;
                                dwParse++;
                            } break;
                        }
                    } break;
                default:
                    {
                        pszFormat++;
                        dwParse++;
                    } break;
                }
            } break;
        default:
            {
                /* normal char */
                pszWork[0] = pszFormat[0];
                pszWork++;
                pszFormat++;
                dwParse++;
            } break;
        }
    }

    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "DIAG(%s): %s\n", pszSeverity, szOutPut));

Exit:
    return;
}
#endif /* INCLUDE_MASTER_OBD */

/********************************************************************************/
/** \brief  Read object dictionary.
 *
 * This function reads the CoE object dictionary.
 *
 * \return EC_E_NOERROR on success, error code otherwise.
 */
EC_T_DWORD CoeReadObjectDictionary(
    T_EC_DEMO_APP_CONTEXT* pAppContext,
    EC_T_BOOL*           pbStopReading,    /**< [in]   Pointer to shutdwon flag */
    EC_T_DWORD           dwNodeId,         /**< [in]   Slave Id to query ODL from  */
    EC_T_BOOL            bPerformUpload,   /**< [in]   EC_TRUE: do SDO Upload */
    EC_T_DWORD           dwTimeout         /**< [in]   Individual call timeout */
)
{
    /* buffer sizes */
#define CROD_ODLTFER_SIZE       ((EC_T_DWORD)0x1200)
#define CROD_OBDESC_SIZE        ((EC_T_DWORD)100)
#define CROD_ENTRYDESC_SIZE     ((EC_T_DWORD)100)
#define CROD_MAXSISDO_SIZE      ((EC_T_DWORD)0x200)
#define MAX_OBNAME_LEN          ((EC_T_DWORD)100)

    /* variables */
    EC_T_DWORD          dwRetVal                = EC_E_ERROR;   /* return value */
    EC_T_DWORD          dwRes                   = EC_E_ERROR;   /* tmp return value for API calls */
    EC_T_DWORD          dwClientId              = 0;
    EC_T_CHAR*          szLogBuffer             = EC_NULL;      /* log buffer for formatted string*/
    EC_T_BYTE*          pbyODLTferBuffer        = EC_NULL;      /* OD List */
    EC_T_BYTE*          pbyOBDescTferBuffer     = EC_NULL;      /* OB Desc */
    EC_T_BYTE*          pbyGetEntryTferBuffer   = EC_NULL;      /* Entry Desc */
    EC_T_MBXTFER_DESC   oMbxTferDesc;                           /* mailbox transfer descriptor */
    EC_T_MBXTFER*       pMbxGetODLTfer          = EC_NULL;      /* mailbox transfer object for OD list upload */
    EC_T_MBXTFER*       pMbxGetObDescTfer       = EC_NULL;      /* mailbox transfer object for Object description upload */
    EC_T_MBXTFER*       pMbxGetEntryDescTfer    = EC_NULL;      /* mailbox transfer object for Entry description upload */

    EC_T_WORD*          pwODList                = EC_NULL;      /* is going to carry ALL list of OD */
    EC_T_WORD           wODListLen              = 0;            /* used entries in pwODList */
    EC_T_WORD           wIndex                  = 0;            /* index to parse OD List */

    EC_T_BYTE           byValueInfoType         = 0;
    EC_T_DWORD          dwUniqueTransferId      = 0;
    EC_T_BOOL           bReadingMasterOD        = EC_FALSE;

    OsMemset(&oMbxTferDesc, 0, sizeof(EC_T_MBXTFER_DESC));

    /* Check Parameters */
    if ((EC_NULL == pAppContext) || (EC_NULL == pbStopReading) || (EC_NOWAIT == dwTimeout))
    {
        dwRetVal = EC_E_INVALIDPARM;
        goto Exit;
    }
    dwClientId = pAppContext->pNotificationHandler->GetClientID();

    /* Create Memory */
    pbyODLTferBuffer        = (EC_T_BYTE*)OsMalloc(CROD_ODLTFER_SIZE);
    pbyOBDescTferBuffer     = (EC_T_BYTE*)OsMalloc(CROD_OBDESC_SIZE);
    pbyGetEntryTferBuffer   = (EC_T_BYTE*)OsMalloc(CROD_ENTRYDESC_SIZE);
    szLogBuffer             = (EC_T_CHAR*)OsMalloc(LOG_BUFFER_SIZE);

    szLogBuffer[0] = '\0';
    szLogBuffer[LOG_BUFFER_SIZE - 1] = '\0';

    /* check if alloc was ok */
    if ((EC_NULL == pbyODLTferBuffer)
     || (EC_NULL == pbyOBDescTferBuffer)
     || (EC_NULL == pbyGetEntryTferBuffer))
    {
        dwRetVal = EC_E_NOMEMORY;
        goto Exit;
    }

    OsMemset(pbyODLTferBuffer,      0, CROD_ODLTFER_SIZE);
    OsMemset(pbyOBDescTferBuffer,   0, CROD_OBDESC_SIZE);
    OsMemset(pbyGetEntryTferBuffer, 0, CROD_ENTRYDESC_SIZE);

    /* create required MBX Transfer Objects */
    /* mailbox transfer object for OD list upload */
    oMbxTferDesc.dwMaxDataLen        = CROD_ODLTFER_SIZE;
    oMbxTferDesc.pbyMbxTferDescData  = pbyODLTferBuffer;

    pMbxGetODLTfer = emMbxTferCreate(pAppContext->dwInstanceId, &oMbxTferDesc);
    if (EC_NULL == pMbxGetODLTfer)
    {
        dwRetVal = EC_E_NOMEMORY;
        goto Exit;
    }

    /* mailbox transfer object for Object description upload */
    oMbxTferDesc.dwMaxDataLen        = CROD_OBDESC_SIZE;
    oMbxTferDesc.pbyMbxTferDescData  = pbyOBDescTferBuffer;

    pMbxGetObDescTfer = emMbxTferCreate(pAppContext->dwInstanceId, &oMbxTferDesc);
    if (EC_NULL == pMbxGetObDescTfer)
    {
        dwRetVal = EC_E_NOMEMORY;
        goto Exit;
    }

    /* mailbox transfer object for Entry description upload */
    oMbxTferDesc.dwMaxDataLen        = CROD_ENTRYDESC_SIZE;
    oMbxTferDesc.pbyMbxTferDescData  = pbyGetEntryTferBuffer;

    pMbxGetEntryDescTfer = emMbxTferCreate(pAppContext->dwInstanceId, &oMbxTferDesc);
    if (EC_NULL == pMbxGetEntryDescTfer)
    {
        dwRetVal = EC_E_NOMEMORY;
        goto Exit;
    }

    /* Get OD List Type: ALL */
    pMbxGetODLTfer->dwClntId    = dwClientId;
    pMbxGetODLTfer->dwTferId    = dwUniqueTransferId++;
    pMbxGetODLTfer->dwDataLen   = pMbxGetODLTfer->MbxTferDesc.dwMaxDataLen;

    /* get list of object indexes */
    dwRes = emCoeGetODList(pAppContext->dwInstanceId, pMbxGetODLTfer, dwNodeId, eODListType_ALL, dwTimeout);
    if (EC_E_SLAVE_NOT_PRESENT == dwRes)
    {
        dwRetVal = dwRes;
        goto Exit;
    }

    /* wait until transfer object is available incl. logging error */
    HandleMbxTferReqError(pAppContext, "CoeReadObjectDictionary: Error in emCoeGetODList(ALL)", dwRes, pMbxGetODLTfer);
    if (EC_E_NOERROR != dwRes)
    {
        dwRetVal = dwRes;
        goto Exit;
    }

    if (0 == pMbxGetODLTfer->MbxData.CoE_ODList.wLen)
    {
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "emCoeGetODList didn't return any objects\n"));
        dwRetVal = EC_E_NOERROR;
        goto Exit;
    }

    /* OD Tfer object now shall contain complete list of OD Objects, store it for more processing */
    pwODList = (EC_T_WORD*)OsMalloc(sizeof(EC_T_WORD) * pMbxGetODLTfer->MbxData.CoE_ODList.wLen);
    if (EC_NULL == pwODList)
    {
        dwRetVal = EC_E_NOMEMORY;
        goto Exit;
    }
    OsMemset(pwODList, 0, sizeof(EC_T_WORD) * pMbxGetODLTfer->MbxData.CoE_ODList.wLen);

    /* reading master OD */
    if (MASTER_SLAVE_ID == dwNodeId)
    {
        bReadingMasterOD = EC_TRUE;
    }

    /* now display Entries of ODList and store non-empty values */
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "Complete OD list:\n"));

    /* iterate through all entries in list */
    for (wODListLen = 0, wIndex = 0; wIndex < (pMbxGetODLTfer->MbxData.CoE_ODList.wLen); wIndex++)
    {
        /* store next index */
        pwODList[wODListLen] = pMbxGetODLTfer->MbxData.CoE_ODList.pwOdList[wIndex];

        /* show indices */
        if (pAppContext->AppParms.dwAppLogLevel >= EC_LOG_LEVEL_INFO)
        {
            OsSnprintf(&szLogBuffer[OsStrlen(szLogBuffer)], (EC_T_INT)(LOG_BUFFER_SIZE - OsStrlen(szLogBuffer) - 1), "%04X ", pwODList[wODListLen]);
            if (((wIndex + 1) % 10) == 0) FlushLogBuffer(pAppContext, szLogBuffer);
        }

        /* to store only non empty index entries, increment List Length only if non zero entry */
        if (0 != pwODList[wODListLen])
        {
            wODListLen++;
        }
    }
    FlushLogBuffer(pAppContext, szLogBuffer);

    /* MbxGetODLTfer done */
    pMbxGetODLTfer->eTferStatus = eMbxTferStatus_Idle;

    /* Get OD List Type: RX PDO Map */
    pMbxGetODLTfer->dwClntId    = dwClientId;
    pMbxGetODLTfer->dwTferId    = dwUniqueTransferId++;
    pMbxGetODLTfer->dwDataLen   = pMbxGetODLTfer->MbxTferDesc.dwMaxDataLen;

    dwRes = emCoeGetODList(pAppContext->dwInstanceId, pMbxGetODLTfer, dwNodeId, eODListType_RxPdoMap, dwTimeout);
    if (EC_E_SLAVE_NOT_PRESENT == dwRes)
    {
        dwRetVal = dwRes;
        goto Exit;
    }

    /* wait until transfer object is available incl. logging error */
    HandleMbxTferReqError(pAppContext, "CoeReadObjectDictionary: Error in emCoeGetODList(RxPdoMap)", dwRes, pMbxGetODLTfer);
    if (EC_E_NOERROR != dwRes)
    {
        dwRetVal = dwRes;
        goto Exit;
    }

    if (pAppContext->AppParms.dwAppLogLevel >= EC_LOG_LEVEL_INFO)
    {
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "RX PDO Mappable Objects:\n"));
        /* iterate through all entries in list */
        for (wIndex = 0; wIndex < (pMbxGetODLTfer->MbxData.CoE_ODList.wLen); wIndex++)
        {
            OsSnprintf(&szLogBuffer[OsStrlen(szLogBuffer)], (EC_T_INT)(LOG_BUFFER_SIZE - OsStrlen(szLogBuffer) - 1), "%04X ", pMbxGetODLTfer->MbxData.CoE_ODList.pwOdList[wIndex]);
            if (((wIndex+1) % 10) == 0) FlushLogBuffer(pAppContext, szLogBuffer);
        }
        FlushLogBuffer(pAppContext, szLogBuffer);
    }
    /* MbxGetODLTfer done */
    pMbxGetODLTfer->eTferStatus = eMbxTferStatus_Idle;

    /* Get OD List Type: TX PDO Map */
    pMbxGetODLTfer->dwClntId    = dwClientId;
    pMbxGetODLTfer->dwTferId    = dwUniqueTransferId++;
    pMbxGetODLTfer->dwDataLen   = pMbxGetODLTfer->MbxTferDesc.dwMaxDataLen;

    dwRes = emCoeGetODList(pAppContext->dwInstanceId, pMbxGetODLTfer, dwNodeId, eODListType_TxPdoMap, dwTimeout);
    if (EC_E_SLAVE_NOT_PRESENT == dwRes)
    {
        dwRetVal = dwRes;
        goto Exit;
    }

    /* wait until transfer object is available incl. logging error */
    HandleMbxTferReqError(pAppContext, "CoeReadObjectDictionary: Error in emCoeGetODList(TxPdoMap)", dwRes, pMbxGetODLTfer);
    if (EC_E_NOERROR != dwRes)
    {
        dwRetVal = dwRes;
        goto Exit;
    }

    /* now display Entries of ODList */
    if (pAppContext->AppParms.dwAppLogLevel >= EC_LOG_LEVEL_INFO)
    {
        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "TX PDO Mappable Objects:\n"));
        /* iterate through all entries in list */
        for( wIndex = 0; wIndex < (pMbxGetODLTfer->MbxData.CoE_ODList.wLen); wIndex++ )
        {
            OsSnprintf(&szLogBuffer[OsStrlen(szLogBuffer)], (EC_T_INT)(LOG_BUFFER_SIZE - OsStrlen(szLogBuffer) - 1), "%04X ", pMbxGetODLTfer->MbxData.CoE_ODList.pwOdList[wIndex]);

            if (((wIndex + 1) % 10) == 0) FlushLogBuffer(pAppContext, szLogBuffer);
        }
        FlushLogBuffer(pAppContext, szLogBuffer);
    }
    /* MbxGetODLTfer done */
    pMbxGetODLTfer->eTferStatus = eMbxTferStatus_Idle;

    /* now iterate through Index list, to get closer info, sub indexes and values */

    /* get object description for all objects */
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "\n"));
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "*************************************************************\n"));
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "****                  OBJECT DESCRIPTION                 ****\n"));
    EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "*************************************************************\n"));

    /* init value info type */
    byValueInfoType = EC_COE_ENTRY_ObjAccess
                    | EC_COE_ENTRY_ObjCategory
                    | EC_COE_ENTRY_PdoMapping
                    | EC_COE_ENTRY_UnitType
                    | EC_COE_ENTRY_DefaultValue
                    | EC_COE_ENTRY_MinValue
                    | EC_COE_ENTRY_MaxValue;

    for (wIndex = 0; (wIndex < wODListLen) && !*pbStopReading; wIndex++)
    {
        EC_T_WORD wSubIndexLimit = 0x100; /* SubIndex range: 0x00 ... 0xff */
        EC_T_WORD wSubIndex      = 0;

        /* get Object Description */
        pMbxGetObDescTfer->dwClntId     = dwClientId;
        pMbxGetObDescTfer->dwDataLen    = pMbxGetObDescTfer->MbxTferDesc.dwMaxDataLen;
        pMbxGetObDescTfer->dwTferId     = dwUniqueTransferId++;

        /* get object description */
        dwRes = emCoeGetObjectDesc(pAppContext->dwInstanceId, pMbxGetObDescTfer, dwNodeId, pwODList[wIndex], dwTimeout);
        if (EC_E_SLAVE_NOT_PRESENT == dwRes)
        {
            dwRetVal = dwRes;
            goto Exit;
        }

        /* wait until transfer object is available incl. logging error */
        HandleMbxTferReqError(pAppContext, "CoeReadObjectDictionary: Error in emCoeGetObjectDesc", dwRes, pMbxGetODLTfer);
        if (EC_E_NOERROR != dwRes)
        {
            dwRetVal = dwRes;
            goto Exit;
        }

        /* display ObjectDesc */
        if (pAppContext->AppParms.dwAppLogLevel >= EC_LOG_LEVEL_INFO)
        {
            EC_T_WORD wNameLen = 0;
            EC_T_CHAR szObName[MAX_OBNAME_LEN];

            wNameLen = pMbxGetObDescTfer->MbxData.CoE_ObDesc.wObNameLen;
            wNameLen = EC_AT_MOST(wNameLen, (EC_T_WORD)(MAX_OBNAME_LEN - 1));

            OsStrncpy(szObName, pMbxGetObDescTfer->MbxData.CoE_ObDesc.pchObName, (EC_T_INT)wNameLen);
            szObName[wNameLen] = '\0';

            EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "%04x %s: type 0x%04x, code=0x%02x, %s, SubIds=%d",
                pMbxGetObDescTfer->MbxData.CoE_ObDesc.wObIndex,
                szObName,
                pMbxGetObDescTfer->MbxData.CoE_ObDesc.wDataType,
                pMbxGetObDescTfer->MbxData.CoE_ObDesc.byObjCode,
                ((pMbxGetObDescTfer->MbxData.CoE_ObDesc.byObjCategory == 0) ? "optional" : "mandatory"),
                pMbxGetObDescTfer->MbxData.CoE_ObDesc.byMaxNumSubIndex));

            /* give logging task a chance to flush */
            if (bReadingMasterOD)
                OsSleep(2);
        }

        /* if Object is Single Variable, only subindex 0 is defined */
        if (OBJCODE_VAR == pMbxGetObDescTfer->MbxData.CoE_ObDesc.byObjCode)
        {
            wSubIndexLimit = 1;
        }
        else
        {
            wSubIndexLimit = 0x100;
        }

        /* iterate through sub-indexes */
        for (wSubIndex = 0; wSubIndex < wSubIndexLimit; wSubIndex++)
        {
            /* Get Entry Description */
            pMbxGetEntryDescTfer->dwClntId     = dwClientId;
            pMbxGetEntryDescTfer->dwDataLen    = pMbxGetEntryDescTfer->MbxTferDesc.dwMaxDataLen;
            pMbxGetEntryDescTfer->dwTferId     = dwUniqueTransferId++;

            dwRes = emCoeGetEntryDesc(pAppContext->dwInstanceId, pMbxGetEntryDescTfer, dwNodeId, pwODList[wIndex], EC_LOBYTE(wSubIndex), byValueInfoType, dwTimeout);
            if (EC_E_SLAVE_NOT_PRESENT == dwRes)
            {
                dwRetVal = dwRes;
                goto Exit;
            }

            /* break after last index */
            if ((EC_E_INVALIDDATA == dwRes) || (EC_E_SDO_ABORTCODE_OFFSET == dwRes))
            {
                break;
            }

            /* handle MBX Tfer errors and wait until tfer object is available */
            HandleMbxTferReqError(pAppContext, "CoeReadObjectDictionary: Error in emCoeGetEntryDesc", dwRes, pMbxGetEntryDescTfer);

            /* display EntryDesc */
            {
                EC_T_CHAR   szAccess[50];
                EC_T_INT    nAccessIdx          = 0;
                EC_T_CHAR   szPdoMapInfo[50];
                EC_T_INT    nDataIdx            = 0;
                EC_T_CHAR   szUnitType[50];
                EC_T_CHAR   szDefaultValue[10];
                EC_T_CHAR   szMinValue[10];
                EC_T_CHAR   szMaxValue[10];
                EC_T_CHAR   szDescription[50];

                EC_T_DWORD  dwUnitType          = 0;
                EC_T_BYTE*  pbyDefaultValue     = EC_NULL;
                EC_T_BYTE*  pbyMinValue         = EC_NULL;
                EC_T_BYTE*  pbyMaxValue         = EC_NULL;

                OsMemset(szAccess, 0, sizeof(szAccess));
                OsMemset(szPdoMapInfo, 0, sizeof(szPdoMapInfo));
                OsMemset(szUnitType, 0, sizeof(szUnitType));
                OsMemset(szDefaultValue, 0, sizeof(szDefaultValue));
                OsMemset(szMinValue, 0, sizeof(szMinValue));
                OsMemset(szMaxValue, 0, sizeof(szMaxValue));
                OsMemset(szDescription, 0, sizeof(szDescription));

                /* build Access Right String */
                if (pMbxGetEntryDescTfer->MbxData.CoE_EntryDesc.byObAccess & EC_COE_ENTRY_Access_R_PREOP)
                    szAccess[nAccessIdx++] = 'R';
                else
                    szAccess[nAccessIdx++] = ' ';
                if (pMbxGetEntryDescTfer->MbxData.CoE_EntryDesc.byObAccess & EC_COE_ENTRY_Access_W_PREOP)
                    szAccess[nAccessIdx++] = 'W';
                else
                    szAccess[nAccessIdx++] = ' ';

                szAccess[nAccessIdx++] = '.';

                if (pMbxGetEntryDescTfer->MbxData.CoE_EntryDesc.byObAccess & EC_COE_ENTRY_Access_R_SAFEOP)
                    szAccess[nAccessIdx++] = 'R';
                else
                    szAccess[nAccessIdx++] = ' ';
                if (pMbxGetEntryDescTfer->MbxData.CoE_EntryDesc.byObAccess & EC_COE_ENTRY_Access_W_SAFEOP)
                    szAccess[nAccessIdx++] = 'W';
                else
                    szAccess[nAccessIdx++] = ' ';

                szAccess[nAccessIdx++] = '.';

                if (pMbxGetEntryDescTfer->MbxData.CoE_EntryDesc.byObAccess & EC_COE_ENTRY_Access_R_OP)
                    szAccess[nAccessIdx++] = 'R';
                else
                    szAccess[nAccessIdx++] = ' ';
                if (pMbxGetEntryDescTfer->MbxData.CoE_EntryDesc.byObAccess & EC_COE_ENTRY_Access_W_OP)
                    szAccess[nAccessIdx++] = 'W';
                else
                    szAccess[nAccessIdx++] = ' ';

                if (pMbxGetEntryDescTfer->MbxData.CoE_EntryDesc.bRxPdoMapping)
                {
                    OsStrncpy(szPdoMapInfo, "-RxPDO", sizeof(szPdoMapInfo) - 1);
                    if (pMbxGetEntryDescTfer->MbxData.CoE_EntryDesc.bTxPdoMapping)
                    {
                        OsStrncpy(&(szPdoMapInfo[OsStrlen(szPdoMapInfo)]), "+TxPDO", sizeof(szPdoMapInfo) - OsStrlen(szPdoMapInfo) - 1);
                    }
                }

                if (pMbxGetEntryDescTfer->MbxData.CoE_EntryDesc.byValueInfo & EC_COE_ENTRY_UnitType)
                {
                    dwUnitType = EC_GET_FRM_DWORD(pMbxGetEntryDescTfer->MbxData.CoE_EntryDesc.pbyData);
                    OsSnprintf(szUnitType, sizeof(szUnitType) - 1, ", UnitType 0x%08X", dwUnitType);
                    nDataIdx += 4;
                }

                if (pMbxGetEntryDescTfer->MbxData.CoE_EntryDesc.byValueInfo & EC_COE_ENTRY_DefaultValue)
                {
                    OsStrncpy(szDefaultValue, ", Default", sizeof(szDefaultValue) - 1);
                    pbyDefaultValue = &pMbxGetEntryDescTfer->MbxData.CoE_EntryDesc.pbyData[nDataIdx];
                    nDataIdx += BIT2BYTE(pMbxGetEntryDescTfer->MbxData.CoE_EntryDesc.wBitLen);
                }

                if (pMbxGetEntryDescTfer->MbxData.CoE_EntryDesc.byValueInfo & EC_COE_ENTRY_MinValue)
                {
                    OsStrncpy(szMinValue, ", Min", sizeof(szMinValue) - 1);
                    pbyMinValue = &pMbxGetEntryDescTfer->MbxData.CoE_EntryDesc.pbyData[nDataIdx];
                    nDataIdx += BIT2BYTE(pMbxGetEntryDescTfer->MbxData.CoE_EntryDesc.wBitLen);
                }

                if (pMbxGetEntryDescTfer->MbxData.CoE_EntryDesc.byValueInfo & EC_COE_ENTRY_MaxValue)
                {
                    OsStrncpy(szMaxValue, ", Max", sizeof(szMaxValue) - 1);
                    pbyMaxValue = &pMbxGetEntryDescTfer->MbxData.CoE_EntryDesc.pbyData[nDataIdx];
                    nDataIdx += BIT2BYTE(pMbxGetEntryDescTfer->MbxData.CoE_EntryDesc.wBitLen);
                }

                if (nDataIdx + 1 <= pMbxGetEntryDescTfer->MbxData.CoE_EntryDesc.wDataLen)
                {
                    OsSnprintf(szDescription, EC_AT_MOST((EC_T_INT)(pMbxGetEntryDescTfer->MbxData.CoE_EntryDesc.wDataLen - nDataIdx + 1), (EC_T_INT)(sizeof(szDescription) - 1)),
                        "%s", &pMbxGetEntryDescTfer->MbxData.CoE_EntryDesc.pbyData[nDataIdx]);
                }

                EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "%04x:%d %s: data type=0x%04x, bit len=%02d, %s%s%s%s%s%s",
                    pMbxGetEntryDescTfer->MbxData.CoE_EntryDesc.wObIndex,
                    pMbxGetEntryDescTfer->MbxData.CoE_EntryDesc.byObSubIndex,
                    szDescription,
                    pMbxGetEntryDescTfer->MbxData.CoE_EntryDesc.wDataType,
                    pMbxGetEntryDescTfer->MbxData.CoE_EntryDesc.wBitLen,
                    szAccess, szPdoMapInfo, szUnitType, szDefaultValue, szMinValue, szMaxValue));

                EC_UNREFPARM(pbyDefaultValue);
                EC_UNREFPARM(pbyMinValue);
                EC_UNREFPARM(pbyMaxValue);

                /* give logging task a chance to flush */
                if (bReadingMasterOD)
                    OsSleep(2);
            } /* display EntryDesc */

            if (0 == pMbxGetEntryDescTfer->MbxData.CoE_EntryDesc.wDataType)
            {
                /* unknown datatype */
                continue;
            }

            /* SDO Upload */
            if (bPerformUpload)
            {
                EC_T_BYTE  abySDOValue[CROD_MAXSISDO_SIZE];
                EC_T_DWORD dwUploadBytes = 0;

                /* prevent from uploading an write-only object */
                if (MASTER_SLAVE_ID != dwNodeId)
                {
                    EC_T_WORD wCurrDevState = 0;
                    EC_T_WORD wReqDevState = 0;
                    dwRes = emGetSlaveState(pAppContext->dwInstanceId, dwNodeId, &wCurrDevState, &wReqDevState);
                    if (EC_E_NOERROR != dwRes)
                    {
                        dwRetVal = dwRes;
                        goto Exit;
                    }
                    if (((DEVICE_STATE_PREOP == (wCurrDevState & DEVICE_STATE_MASK))
                        && (0 == (pMbxGetEntryDescTfer->MbxData.CoE_EntryDesc.byObAccess & EC_COE_ENTRY_Access_R_PREOP)))
                    || ((DEVICE_STATE_SAFEOP == (wCurrDevState & DEVICE_STATE_MASK))
                        && (0 == (pMbxGetEntryDescTfer->MbxData.CoE_EntryDesc.byObAccess & EC_COE_ENTRY_Access_R_SAFEOP)))
                    || ((DEVICE_STATE_OP == (wCurrDevState & DEVICE_STATE_MASK))
                        && (0 == (pMbxGetEntryDescTfer->MbxData.CoE_EntryDesc.byObAccess & EC_COE_ENTRY_Access_R_OP))))
                    {
                        EcLogMsg(EC_LOG_LEVEL_VERBOSE, (pEcLogContext, EC_LOG_LEVEL_VERBOSE,
                            "CoeReadObjectDictionary: skip non-reable entry 0x%04x, SI %d\n", pwODList[wIndex], EC_LOBYTE(wSubIndex)));
                        continue;
                    }
                }

                /* get object's value */
                dwRes = emCoeSdoUpload(
                    pAppContext->dwInstanceId, dwNodeId, pwODList[wIndex], EC_LOBYTE(wSubIndex),
                    abySDOValue, EC_AT_MOST(BIT2BYTE(pMbxGetEntryDescTfer->MbxData.CoE_EntryDesc.wBitLen), (EC_T_WORD)CROD_MAXSISDO_SIZE), &dwUploadBytes, dwTimeout, 0);
                if (EC_E_SLAVE_NOT_PRESENT == dwRes)
                {
                    dwRetVal = dwRes;
                    goto Exit;
                }
                else if (EC_E_NOERROR != dwRes)
                {
                    /* Upload error */
                    EcLogMsg(EC_LOG_LEVEL_ERROR, (pEcLogContext, EC_LOG_LEVEL_ERROR, "CoeReadObjectDictionary: Error in ecatCoeSdoUpload: %s (0x%lx)\n", ecatGetText(dwRes), dwRes));
                    dwRetVal = dwRes;
                    continue;
                }

                if (((OBJCODE_REC == pMbxGetObDescTfer->MbxData.CoE_ObDesc.byObjCode) && (0 == wSubIndex))
                 || ((OBJCODE_ARR == pMbxGetObDescTfer->MbxData.CoE_ObDesc.byObjCode) && (0 == wSubIndex)))
                {
                    wSubIndexLimit = (EC_T_WORD)(((EC_T_WORD)abySDOValue[0]) + 1);
                }

                switch (pMbxGetEntryDescTfer->MbxData.CoE_EntryDesc.wDataType)
                {
                case DEFTYPE_BOOLEAN:
                case DEFTYPE_BIT1:
                case DEFTYPE_BIT2:
                case DEFTYPE_BIT3:
                case DEFTYPE_BIT4:
                case DEFTYPE_BIT5:
                case DEFTYPE_BIT6:
                case DEFTYPE_BIT7:
                case DEFTYPE_BIT8:
                case DEFTYPE_INTEGER8:
                case DEFTYPE_UNSIGNED8:
                    {
                        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "%04x:%d BYTE: 0x%02X",
                            pwODList[wIndex], EC_LOBYTE(wSubIndex), abySDOValue[0]));
                    } break;
                case DEFTYPE_INTEGER16:
                    {
                        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "%04x:%d SI16: %04d",
                            pwODList[wIndex], EC_LOBYTE(wSubIndex), EC_GET_FRM_WORD(&abySDOValue[0])));
                    } break;
                case DEFTYPE_BIT9:
                case DEFTYPE_BIT10:
                case DEFTYPE_BIT11:
                case DEFTYPE_BIT12:
                case DEFTYPE_BIT13:
                case DEFTYPE_BIT14:
                case DEFTYPE_BIT15:
                case DEFTYPE_BIT16:
                case DEFTYPE_UNSIGNED16:
                    {
                        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "%04x:%d WORD: 0x%04X",
                            pwODList[wIndex], EC_LOBYTE(wSubIndex), EC_GET_FRM_WORD(&abySDOValue[0])));
                    } break;
                case DEFTYPE_INTEGER32:
                    {
                        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "%04x:%d SI32: %08ld",
                            pwODList[wIndex], EC_LOBYTE(wSubIndex), EC_GET_FRM_DWORD(&abySDOValue[0])));
                    } break;
                case DEFTYPE_UNSIGNED32:
                    {
                        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "%04x:%d DWRD: 0x%08lX",
                            pwODList[wIndex], EC_LOBYTE(wSubIndex), EC_GET_FRM_DWORD(&abySDOValue[0])));
                    } break;
                case DEFTYPE_VISIBLESTRING:
                    {

                        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "%04x:%d STRG: %s",
                            pwODList[wIndex], EC_LOBYTE(wSubIndex), (EC_T_CHAR*)abySDOValue));
                    } break;
                case DEFTYPE_OCTETSTRING:
                    {
#if (defined INCLUDE_MASTER_OBD)
                        if( (COEOBJID_HISTORY_OBJECT == (pwODList[wIndex])) && EC_LOBYTE(wSubIndex) > 5 )
                        {
                            /* Diag Entry */
                            EC_T_OBJ10F3_DIAGMSG*   pDiag = (EC_T_OBJ10F3_DIAGMSG*)abySDOValue;
                            EC_T_DWORD dwTimeStampHi = EC_HIDWORD(pDiag->qwTimeStamp);
                            EC_T_DWORD dwTimeStampLo = EC_LODWORD(pDiag->qwTimeStamp);

                            EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "%04x:%d DIAG # 0x%lx type <%s> Text 0x%x Time: 0x%x.%x",
                                pwODList[wIndex], EC_LOBYTE(wSubIndex),
                                pDiag->dwDiagNumber,
                                (((pDiag->wFlags & 0x0F) == DIAGFLAGERROR) ? "ERR" : (((pDiag->wFlags & 0x0F) == DIAGFLAGWARN) ? "WARN" : (((pDiag->wFlags & 0x0F) == DIAGFLAGINFO) ? "INF" : "UNK"))),
                                pDiag->wTextId, dwTimeStampHi, dwTimeStampLo));
                            ParseDiagMsg(pAppContext, pDiag);
                        }
                        else
#endif /* INCLUDE_MASTER_OBD */
                        {
                            EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "%04x:%d OCTS: %s",
                                pwODList[wIndex], EC_LOBYTE(wSubIndex), (EC_T_CHAR*)abySDOValue));
                        }
                    } break;
                case DEFTYPE_UNSIGNED48:
                    {
                        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "%04x:%d US48: %02X:%02X:%02X:%02X:%02X:%02X",
                            pwODList[wIndex], EC_LOBYTE(wSubIndex),
                            abySDOValue[0],
                            abySDOValue[1],
                            abySDOValue[2],
                            abySDOValue[3],
                            abySDOValue[4],
                            abySDOValue[5]));
                    } break;
                case DEFTYPE_UNSIGNED64:
                    {
                        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "%04x:%d QWRD: 0x%08lX.%08lX\n",
                            pwODList[wIndex], EC_LOBYTE(wSubIndex),
                            EC_HIDWORD(EC_GET_FRM_QWORD(&abySDOValue[0])),
                            EC_LODWORD(EC_GET_FRM_QWORD(&abySDOValue[0]))));
                    } break;
                default:
                    {
                        EC_T_DWORD  dwIdx = 0;

                        EcLogMsg(EC_LOG_LEVEL_INFO, (pEcLogContext, EC_LOG_LEVEL_INFO, "%04x:%d DFLT: \n", pwODList[wIndex], EC_LOBYTE(wSubIndex)));
                        for (dwIdx = 0; dwIdx < dwUploadBytes; dwIdx++)
                        {
                            OsSnprintf(&szLogBuffer[OsStrlen(szLogBuffer)], (EC_T_INT)(LOG_BUFFER_SIZE - OsStrlen(szLogBuffer) - 1), "%02x ", abySDOValue[dwIdx]);
                            if ((0 != dwIdx) && (0 == (dwIdx%8)))
                            {
                                OsSnprintf(&szLogBuffer[OsStrlen(szLogBuffer)], (EC_T_INT)(LOG_BUFFER_SIZE - OsStrlen(szLogBuffer) - 1), "%s", " ");
                            }
                            if ((0 != dwIdx) && (0 == (dwIdx % 32))) FlushLogBuffer(pAppContext, szLogBuffer);
                        }
                        FlushLogBuffer(pAppContext, szLogBuffer);
                    } break;
                } /* switch (pMbxGetEntryDescTfer->MbxData.CoE_EntryDesc.wDataType) */

#if (defined INCLUDE_MASTER_OBD)
                if (COEOBJID_SLAVECFGINFOBASE <= pwODList[wIndex] && 1 == EC_LOBYTE(wSubIndex))
                {
                    EC_T_BOOL bEntryValid   = EC_FALSE;
                    bEntryValid = EC_GET_FRM_BOOL(abySDOValue);

                    /* do not show unused Slave Entries */
                    if (!bEntryValid)
                    {
                        break;
                    }
                }
#endif /* INCLUDE_MASTER_OBD */
                /* give logging task a chance to flush */
                if (bReadingMasterOD)
                    OsSleep(2);
            } /* bPerformUpload */

            /* MbxGetObDescTfer done */
            pMbxGetObDescTfer->eTferStatus = eMbxTferStatus_Idle;
        } /* for (wSubIndex = 0; wSubIndex < wSubIndexLimit; wSubIndex++) */
    } /* for (wIndex = 0; (wIndex < wODListLen) && !*pbStopReading; wIndex++) */

    dwRetVal = EC_E_NOERROR;

Exit:
    /* Delete MBX Transfer objects */
    if (EC_NULL != pMbxGetODLTfer)
    {
        emMbxTferDelete(pAppContext->dwInstanceId, pMbxGetODLTfer);
        pMbxGetODLTfer = EC_NULL;
    }
    if (EC_NULL != pMbxGetObDescTfer)
    {
        emMbxTferDelete(pAppContext->dwInstanceId, pMbxGetObDescTfer);
        pMbxGetObDescTfer = EC_NULL;
    }
    if (EC_NULL != pMbxGetEntryDescTfer)
    {
        emMbxTferDelete(pAppContext->dwInstanceId, pMbxGetEntryDescTfer);
        pMbxGetEntryDescTfer = EC_NULL;
    }

    /* Free allocated memory */
    SafeOsFree(pwODList);
    SafeOsFree(pbyODLTferBuffer);
    SafeOsFree(pbyOBDescTferBuffer);
    SafeOsFree(pbyGetEntryTferBuffer);
    SafeOsFree(szLogBuffer);

    return dwRetVal;
}
/*-END OF SOURCE FILE--------------------------------------------------------*/
