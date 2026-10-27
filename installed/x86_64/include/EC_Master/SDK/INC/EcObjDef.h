/*-----------------------------------------------------------------------------
 * EcObjDef.h
 * Copyright                acontis technologies GmbH, Ravensburg, Germany
 * Response                 Paul Bussmann
 * Description              Master OD structure
 *---------------------------------------------------------------------------*/

#ifndef INC_ECOBJDEF
#define INC_ECOBJDEF 1

/*-INCLUDES------------------------------------------------------------------*/
#ifndef INC_ECOS
#include "EcOs.h"
#endif

/*-DEFINES-------------------------------------------------------------------*/
/*EC_T_SDOINFOOBJDESC*/
#define OBJFLAGS_MAXSUBINDEXMASK      0x00FF
#define OBJFLAGS_MAXSUBINDEXSHIFT     0
#define OBJFLAGS_OBJCODEMASK          0x0F00
#define OBJFLAGS_OBJCODESHIFT         8
#define OBJCODE_VAR                   0x07
#define OBJCODE_ARR                   0x08
#define OBJCODE_REC                   0x09

/*EC_T_SDOINFOENTRYDESC*/
#define ACCESS_READWRITE              0x003F
#define ACCESS_READ                   0x0007
#define ACCESS_READ_PREOP             0x0001
#define ACCESS_READ_SAFEOP            0x0002
#define ACCESS_READ_OP                0x0004
#define ACCESS_WRITE                  0x0038
#define ACCESS_WRITE_PREOP            0x0008
#define ACCESS_WRITE_SAFEOP           0x0010
#define ACCESS_WRITE_OP               0x0020
#define OBJACCESS_NOPDOMAPPING        0x0000
#define OBJACCESS_RXPDOMAPPING        0x0040
#define OBJACCESS_TXPDOMAPPING        0x0080
#define OBJACCESS_BACKUP              0x0100
#define OBJACCESS_SETTINGS            0x0200
#define OBJACCESS_FORCE               0x8000

/* SDO indexes */
#define COEOBJID_DEVICE_TYPE        ((EC_T_WORD)0x1000)
#define COEOBJID_MANF_DEVICE_NAME   ((EC_T_WORD)0x1008)
#define COEOBJID_MANF_HW_VERSION    ((EC_T_WORD)0x1009)
#define COEOBJID_MANF_SW_VERSION    ((EC_T_WORD)0x100A)
#define COEOBJID_IDENTITY_OBJECT    ((EC_T_WORD)0x1018)

#define COEOBJID_ENDLIST            ((EC_T_WORD)0xFFFF)

/*-TYPEDEFS------------------------------------------------------------------*/
/* Identity */
#include EC_PACKED_INCLUDESTART(1)
typedef struct _EC_T_OBJ1018
{
    EC_T_WORD   wSubIndex0;         /* Subindex 000 */
    EC_T_DWORD  dwVendorID;         /* Subindex 001 */
    EC_T_DWORD  dwProductcode;      /* Subindex 002 */
    EC_T_DWORD  dwRevision;         /* Subindex 003 */
    EC_T_DWORD  dwSerialnumber;     /* Subindex 004 */
} EC_PACKED(1) EC_T_OBJ1018;
#include EC_PACKED_INCLUDESTOP

#endif /* INC_ECOBJDEF */

/*-END OF SOURCE FILE--------------------------------------------------------*/
