/*-----------------------------------------------------------------------------
 * EcCommon.h
 * Copyright                acontis technologies GmbH, Ravensburg, Germany
 * Response                 Stefan Zintgraf
 * Description              Common header shared by all AT-EM layers.
 *---------------------------------------------------------------------------*/

#ifndef INC_ECCOMMON
#define INC_ECCOMMON 1

/*-INCLUDES------------------------------------------------------------------*/
#ifndef INC_ECOS
#include "EcOs.h"
#endif

/*-DEFINES-------------------------------------------------------------------*/
/* legacy */
#ifndef EC_FNNAME
#define EC_FNNAME ATECAT_API
#endif
#ifndef EC_FNCALL
#define EC_FNCALL
#endif

/* debug tracing */
#define     EC_TRACE_OS                     0x00000001  /* ecat os layer */
#define     EC_TRACE_CONFIG                 0x00000002  /* ecat config layer */
#define     EC_TRACE_LINK                   0x00000004  /* ecat link layer */
#define     EC_TRACE_CORE                   0x00000008  /* ecat master core layer, general */
#define     EC_TRACE_CORE_QUECMD_SEQ        0x00000010  /* ecat core layer, queued command sequence */
#define     EC_TRACE_CORE_SND_CYCFRAMES     0x00000020  /* ecat core layer, sent cyclic frames */
#define     EC_TRACE_CORE_SND_QUEFRAMES     0x00000040  /* ecat core layer, sent queued frames */
#define     EC_TRACE_CORE_RCV_CYCFRAMES     0x00000080  /* ecat core layer, received cyclic frames */
#define     EC_TRACE_CORE_RCV_QUEFRAMES     0x00000100  /* ecat core layer, received queued frames */
#define     EC_TRACE_CORE_SEQUENCE          0x00000200  /* ecat core layer, general sequence for timing */
#define     EC_TRACE_INTERFACE              0x00000400  /* interface layer */
#define     EC_TRACE_INITCMDS               0x00000800  /* trace ecat init commands */
#define     EC_TRACE_CYCCMD_HASH            0x00001000  /* ecat master core layer, hashing of cyclic commands */
#define     EC_TRACE_CREATE_FRAMES          0x00002000  /* ecat master core layer, creation of frames */

#define     EC_TRACE_CORE_ALL_FRAMES (EC_TRACE_CORE_SND_CYCFRAMES|EC_TRACE_CORE_SND_QUEFRAMES|EC_TRACE_CORE_RCV_CYCFRAMES|EC_TRACE_CORE_RCV_QUEFRAMES)

/* memory usage tracing */
#define     EC_MEMTRACE_STRING          0x00000001  /* memory tracing CString class (config layer) */
#define     EC_MEMTRACE_CONFIG          0x00000002  /* ecat config layer, general */
#define     EC_MEMTRACE_CONFIG_STR      0x00000004  /* ecat config layer, string objects */
#define     EC_MEMTRACE_CONFIG_NODE     0x00000008  /* ecat config layer, xml nodes */
#define     EC_MEMTRACE_CONFIG_DATA     0x00000010  /* ecat config layer, xml data */
#define     EC_MEMTRACE_CONFIG_CMDDESC  0x00000020
#define     EC_MEMTRACE_XMLPARSER       0x00000040  /* ecat xml parser */
#define     EC_MEMTRACE_CORE_FACTORY    0x00000080  /* ecat core layer, device factory */
#define     EC_MEMTRACE_CORE_DEVICE     0x00000100  /* ecat core layer, device */
#define     EC_MEMTRACE_CORE_MASTER     0x00000200  /* ecat core layer, master */
#define     EC_MEMTRACE_CORE_SLAVE      0x00000400  /* ecat core layer, slaves */
#define     EC_MEMTRACE_CORE_SWITCH     0x00000800  /* ecat core layer, switch */
#define     EC_MEMTRACE_CORE_MBX        0x00001000  /* ecat core layer, mailbox */
#define     EC_MEMTRACE_CORE_FRAMES     0x00002000  /* ecat core layer, sent frames */
#define     EC_MEMTRACE_RAS             0x00004000  /* RAS layer */
#define     EC_MEMTRACE_INTERFACE       0x00008000  /* ecat interface layer */
#define     EC_MEMTRACE_APP             0x00010000  /* application */

/*-FUNCTION DECLARATIONS-----------------------------------------------------*/
/* these functions are only available in the DEBUG version */
EC_FNNAME EC_T_DWORD EC_FNCALL ecatGetTraceMask(EC_T_VOID);
EC_FNNAME EC_T_DWORD EC_FNCALL ecatGetMemTraceMask(EC_T_VOID);
EC_FNNAME EC_T_VOID  EC_FNCALL ecatSetTraceMask(EC_T_DWORD dwTraceMask );
EC_FNNAME EC_T_VOID  EC_FNCALL ecatSetMemTraceMask(EC_T_DWORD dwMemTraceMask );

#endif /* INC_ECCOMMON */

/*-END OF SOURCE FILE--------------------------------------------------------*/
