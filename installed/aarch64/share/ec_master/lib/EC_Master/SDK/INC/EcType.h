/** ---------------------------------------------------------------------------
 * \file
 * \brief       EtherCAT Master type definitions
 * \author      Stefan Zintgraf
 * \copyright   acontis technologies GmbH, Weingarten, Germany
 *---------------------------------------------------------------------------*/

#ifndef INC_ECTYPE
#define INC_ECTYPE

/*-TYPEDEFS------------------------------------------------------------------*/
/**
\defgroup EC_TYPES Fundamental data types
@{
*/
/* Use define for EC_T_VOID, otherwise GCC will interpret it as a type and report an incomplete type error */
#define EC_T_VOID void                  /**< Void type */
typedef void*           EC_T_PVOID;     /**< Pointer of type void */

typedef int             EC_T_BOOL;      /**< Boolean */

typedef char            EC_T_CHAR;      /**< Character, 8 bit */
typedef unsigned short  EC_T_WCHAR;     /**< Wide-character, 16 bit */

typedef unsigned char   EC_T_BYTE;      /**< Byte, unsigned integer 8 bit */
typedef unsigned char*  EC_T_PBYTE;     /**< Pointer of type EC_T_BYTE */
typedef unsigned short  EC_T_WORD;      /**< Word, unsigned integer 16 bit */
typedef unsigned int    EC_T_DWORD;     /**< Double word, unsigned integer 32 bit */

typedef signed char     EC_T_SBYTE;     /**< Signed-Byte, signed integer 8 bit */
typedef signed short    EC_T_SWORD;     /**< Signed-Word, signed integer 16 bit */
typedef signed int      EC_T_SDWORD;    /**< Signed-Double-Word, signed integer 32 bit */

typedef int             EC_T_INT;       /**< Integer */
typedef unsigned int    EC_T_UINT;      /**< Unsigned-Integer */

typedef short           EC_T_SHORT;     /**< Short */
typedef unsigned short  EC_T_USHORT;    /**< Unsigned-Short */

typedef float           EC_T_REAL;      /**< Real, floating point */
typedef double          EC_T_LREAL;     /**< long Real, floating point */
/**@}*/

typedef void*           EC_T_ADDRESS;

typedef void*           EC_T_HANDLE;

/* type of lock */
typedef enum
{
    eLockType_DEFAULT= 1,                           /*< Default mutex           */
    eLockType_SPIN,                                 /*< only jobs --> spin lock */
    eLockType_INTERFACE,                            /*< interface and jobs      */

    /* Borland C++ datatype alignment correction */
    eLockType_BCppDummy   = 0xFFFFFFFF
} EC_T_OS_LOCK_TYPE;


/**
\defgroup ECAT_DEFTYPES EtherCAT data types
@{
*/
#define DEFTYPE_NULL              ((EC_T_WORD)0x0000)       /**< Null */
#define DEFTYPE_BOOLEAN           ((EC_T_WORD)0x0001)       /**< Boolean, bit size: 1 */
#define DEFTYPE_INTEGER8          ((EC_T_WORD)0x0002)       /**< Integer, bit size: 8 */
#define DEFTYPE_INTEGER16         ((EC_T_WORD)0x0003)       /**< Integer, bit size: 16 */
#define DEFTYPE_INTEGER32         ((EC_T_WORD)0x0004)       /**< Integer, bit size: 32 */
#define DEFTYPE_UNSIGNED8         ((EC_T_WORD)0x0005)       /**< Unsigned, bit size: 8 */
#define DEFTYPE_UNSIGNED16        ((EC_T_WORD)0x0006)       /**< Unsigned, bit size: 16 */
#define DEFTYPE_UNSIGNED32        ((EC_T_WORD)0x0007)       /**< Unsigned, bit size: 32 */
#define DEFTYPE_REAL32            ((EC_T_WORD)0x0008)       /**< Real, bit size: 32 */
#define DEFTYPE_VISIBLESTRING     ((EC_T_WORD)0x0009)       /**< Visible string, bit size: 8*n */
#define DEFTYPE_OCTETSTRING       ((EC_T_WORD)0x000A)       /**< Octet string, bit size: 8*(n+1) */
#define DEFTYPE_UNICODESTRING     ((EC_T_WORD)0x000B)       /**< Unicode string, bit size: 16*(n+1) */
#define DEFTYPE_TIMEOFDAY         ((EC_T_WORD)0x000C)       /**< Time of day */
#define DEFTYPE_TIMEDIFFERENCE    ((EC_T_WORD)0x000D)       /**< Time difference */
#define DEFTYPE_DOMAIN            ((EC_T_WORD)0x000F)       /**< Domain */
#define DEFTYPE_INTEGER24         ((EC_T_WORD)0x0010)       /**< Integer, bit size: 24 */
#define DEFTYPE_REAL64            ((EC_T_WORD)0x0011)       /**< Real, bit size: 64 */
#define DEFTYPE_INTEGER40         ((EC_T_WORD)0x0012)       /**< Integer, bit size: 40 */
#define DEFTYPE_INTEGER48         ((EC_T_WORD)0x0013)       /**< Integer, bit size: 48 */
#define DEFTYPE_INTEGER56         ((EC_T_WORD)0x0014)       /**< Integer, bit size: 56 */
#define DEFTYPE_INTEGER64         ((EC_T_WORD)0x0015)       /**< Integer, bit size: 64 */
#define DEFTYPE_UNSIGNED24        ((EC_T_WORD)0x0016)       /**< Unsigned, bit size: 24 */
#define DEFTYPE_UNSIGNED40        ((EC_T_WORD)0x0018)       /**< Unsigned, bit size: 40 */
#define DEFTYPE_UNSIGNED48        ((EC_T_WORD)0x0019)       /**< Unsigned, bit size: 48 */
#define DEFTYPE_UNSIGNED56        ((EC_T_WORD)0x001A)       /**< Unsigned, bit size: 56 */
#define DEFTYPE_UNSIGNED64        ((EC_T_WORD)0x001B)       /**< Unsigned, bit size: 64 */

#define DEFTYPE_GUID              ((EC_T_WORD)0x001D)       /**< Guid, bit size: 128 */
#define DEFTYPE_BYTE              ((EC_T_WORD)0x001E)       /**< Byte, bit size: 8 */
#define DEFTYPE_WORD              ((EC_T_WORD)0x001F)       /**< Word, bit size: 16 */
#define DEFTYPE_DWORD             ((EC_T_WORD)0x0020)       /**< Dword, bit size: 32 */
#define DEFTYPE_PDOMAPPING        ((EC_T_WORD)0x0021)       /**< PDO Mapping */

#define DEFTYPE_IDENTITY          ((EC_T_WORD)0x0023)       /**< */

#define DEFTYPE_COMMAND           ((EC_T_WORD)0x0025)       /**< Command */
#define DEFTYPE_PDOCOMPAR         ((EC_T_WORD)0x0027)       /**< PDO COMPAR */
#define DEFTYPE_ENUM              ((EC_T_WORD)0x0028)       /**< Enum */
#define DEFTYPE_SMPAR             ((EC_T_WORD)0x0029)       /**< SMPAR */
#define DEFTYPE_RECORD            ((EC_T_WORD)0x002A)       /**< Record */
#define DEFTYPE_BACKUP_PARAMETER  ((EC_T_WORD)0x002B)       /**< Backup parameter */
#define DEFTYPE_MODULAR_DEVICE_PROFILE ((EC_T_WORD)0x002C)  /**< Modular device profile */
#define DEFTYPE_BITARR8           ((EC_T_WORD)0x002D)       /**< Bit array, bit size: 8 */
#define DEFTYPE_BITARR16          ((EC_T_WORD)0x002E)       /**< Bit array, bit size: 16 */
#define DEFTYPE_BITARR32          ((EC_T_WORD)0x002F)       /**< Bit array, bit size: 32 */
#define DEFTYPE_BIT1              ((EC_T_WORD)0x0030)       /**< Bit, bit size: 1 */
#define DEFTYPE_BIT2              ((EC_T_WORD)0x0031)       /**< Bit, bit size: 2 */
#define DEFTYPE_BIT3              ((EC_T_WORD)0x0032)       /**< Bit, bit size: 3 */
#define DEFTYPE_BIT4              ((EC_T_WORD)0x0033)       /**< Bit, bit size: 4 */
#define DEFTYPE_BIT5              ((EC_T_WORD)0x0034)       /**< Bit, bit size: 5 */
#define DEFTYPE_BIT6              ((EC_T_WORD)0x0035)       /**< Bit, bit size: 6 */
#define DEFTYPE_BIT7              ((EC_T_WORD)0x0036)       /**< Bit, bit size: 7 */
#define DEFTYPE_BIT8              ((EC_T_WORD)0x0037)       /**< Bit, bit size: 8 */
#define DEFTYPE_BIT9              ((EC_T_WORD)0x0038)       /**< Bit, bit size: 9 */
#define DEFTYPE_BIT10             ((EC_T_WORD)0x0039)       /**< Bit, bit size: 10 */
#define DEFTYPE_BIT11             ((EC_T_WORD)0x003A)       /**< Bit, bit size: 11 */
#define DEFTYPE_BIT12             ((EC_T_WORD)0x003B)       /**< Bit, bit size: 12 */
#define DEFTYPE_BIT13             ((EC_T_WORD)0x003C)       /**< Bit, bit size: 13 */
#define DEFTYPE_BIT14             ((EC_T_WORD)0x003D)       /**< Bit, bit size: 14 */
#define DEFTYPE_BIT15             ((EC_T_WORD)0x003E)       /**< Bit, bit size: 15 */
#define DEFTYPE_BIT16             ((EC_T_WORD)0x003F)       /**< Bit, bit size: 16 */

/* 0x40-0x5F Manufacturer Specific Complex Data Type */

#define DEFTYPE_ARRAY_OF_BYTE     ((EC_T_WORD)0x000A)       /**< Array of BYTE, bit size:  8 * (n + 1) */
#define DEFTYPE_ARRAY_OF_UINT     ((EC_T_WORD)0x000B)       /**< Array of UINT, bit size: 16 * (n + 1) */
#define DEFTYPE_ARRAY_OF_INT      ((EC_T_WORD)0x0260)       /**< Array of INT, bit size: 16 * (n + 1) */
#define DEFTYPE_ARRAY_OF_SINT     ((EC_T_WORD)0x0261)       /**< Array of SINT, bit size:  8 * (n + 1) */
#define DEFTYPE_ARRAY_OF_DINT     ((EC_T_WORD)0x0262)       /**< Array of DINT, bit size: 32 * (n + 1) */
#define DEFTYPE_ARRAY_OF_UDINT    ((EC_T_WORD)0x0263)       /**< Array of UDINT, bit size: 32 * (n + 1) */

#define DEFTYPE_ERROR_SETTING     ((EC_T_WORD)0x0281)       /**< Error setting, bit size: - */
#define DEFTYPE_HISTORY           ((EC_T_WORD)0x0282)       /**< History, bit size: - */
#define DEFTYPE_DIAGNOSIS_OBJECT  ((EC_T_WORD)0x0282)       /**< Diagnosis object, bit size: - */
#define DEFTYPE_EXTERNAL_SYNC_STATUS ((EC_T_WORD)0x0283)    /**< External SYNC status, bit size: - */
#define DEFTYPE_EXTERNAL_SYNC_SETTINGS ((EC_T_WORD)0x0284)  /**< External SYNC settings, bit size: - */
#define DEFTYPE_FSOEFRAME         ((EC_T_WORD)0x0285)       /**< FSOE frame, bit size: - */
#define DEFTYPE_FSOECOMMPAR       ((EC_T_WORD)0x0286)       /**< FSOE COMMPAR, bit size: - */
/**@}*/

/*-MACROS--------------------------------------------------------------------*/
#define EC_FALSE            0   /**< Boolean value: False */
#define EC_TRUE             1   /**< Boolean value: True */
#define EC_NULL             0   /**< Null pointer constant */

#define EC_NOWAIT           ((EC_T_DWORD)0x00000000)
#define EC_WAITINFINITE     ((EC_T_DWORD)0xFFFFFFFF)

#define MAX_ERRINFO_STRLEN  8   /* maximum length of error notification info strings */
#define MAX_SHORT_STRLEN    20  /* maximum length of short info string */
#define MAX_STD_STRLEN      80  /* maximum length of standard info string */

/*-MASTER FEATURES-----------------------------------------------------------*/

/* object dictionary */
#define EC_MASTER_DEVICE_NAME       "EC-Master"         /* Index: 0x1008 */
#define EC_MASTER_HW_VERSION        "V xx.xx.xx.xx"     /* Index: 0x1009 */
#define EC_MASTER_SW_VERSION        "V xx.xx.xx.xx"     /* Index: 0x100A */
#define EC_MASTER_VENDOR_ID         0x00004154          /* Index: 0x1018 Subindex 1 */
#define EC_MASTER_PRODUCT_CODE      0x6d657461          /* Index: 0x1018 Subindex 2 */

#define EC_SIMULATOR_DEVICE_NAME    "EC-Simulator"      /* Index: 0x1008 */
#define EC_SIMULATOR_HW_VERSION     "V xx.xx.xx.xx"     /* Index: 0x1009 obsolete. see EC_VERSION_STR */
#define EC_SIMULATOR_SW_VERSION     "V xx.xx.xx.xx"     /* Index: 0x100A obsolete. see EC_VERSION_STR */
#define EC_SIMULATOR_VENDOR_ID      0x00004154          /* Index: 0x1018 Subindex 1 */
#define EC_SIMULATOR_PRODUCT_CODE   0x00000078          /* Index: 0x1018 Subindex 2 */

#define EC_MONITOR_DEVICE_NAME      "EC-Monitor"        /* Index: 0x1008 */
#define EC_MONITOR_HW_VERSION       "V xx.xx.xx.xx"     /* Index: 0x1009 obsolete. see EC_VERSION_STR */
#define EC_MONITOR_SW_VERSION       "V xx.xx.xx.xx"     /* Index: 0x100A obsolete. see EC_VERSION_STR */
#define EC_MONITOR_VENDOR_ID        0x00004154          /* Index: 0x1018 Subindex 1 */
#define EC_MONITOR_PRODUCT_CODE     0x20200321          /* Index: 0x1018 Subindex 2 */

#define EC_ARCH_UNDEFINED 0
#define EC_ARCH_X86       1
#define EC_ARCH_X64       2
#define EC_ARCH_PPC       3
#define EC_ARCH_ARM       4
#define EC_ARCH_ARM64     5
#define EC_ARCH_RISCV64   6
#define EC_ARCH_RX        7

#define EC_OS_UNDEFINED              0
#define EC_OS_WINDOWS                1
#define EC_OS_WINCE                  2
#define EC_OS_RTOS32                 3
#define EC_OS_QNX                    4
#define EC_OS_LINUX                  5
#define EC_OS_INTIME                 6
#define EC_OS_INTEGRITY              7
#define EC_OS_TIRTOS                 8
#define EC_OS_RIN32                  9
#define EC_OS_RTX                   10
#define EC_OS_XILINX_STANDALONE     11
#define EC_OS_ECOS                  12
#define EC_OS_XENOMAI               13
#define EC_OS_UCOS                  14
#define EC_OS_VXWORKS               15
#define EC_OS_SYLIXOS               16
#define EC_OS_UC3                   17
#define EC_OS_FREERTOS              18
#define EC_OS_MACOS                 19
#define EC_OS_CMSIS_RTOS            20
#define EC_OS_ZEPHYR                21
#define EC_OS_RX72                  22

#endif /* INC_ECTYPE */

/*-END OF SOURCE FILE--------------------------------------------------------*/

