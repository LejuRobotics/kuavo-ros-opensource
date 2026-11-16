/*-----------------------------------------------------------------------------
 * EcVersion.h
 * Copyright            acontis technologies GmbH, D-88212 Ravensburg, Germany
 * Description          EC-Master version information
 *---------------------------------------------------------------------------*/

#ifndef INC_ECVERSION
#define INC_ECVERSION 1

/*-DEFINES-------------------------------------------------------------------*/
/* EC_VERSION_TYPE_... */
#define EC_VERSION_TYPE_UNDEFINED    0
#define EC_VERSION_TYPE_UNRESTRICTED 1
#define EC_VERSION_TYPE_PROTECTED    2
#define EC_VERSION_TYPE_DONGLED      3

/*-VERSION INFORMATION-------------------------------------------------------*/
#define EC_VERSION_MAJ               3   /* major version */
#define EC_VERSION_MIN               1   /* minor version */
#define EC_VERSION_SERVICEPACK       4   /* service pack */
#define EC_VERSION_BUILD             7   /* build number */
#define EC_VERSION                   ((EC_VERSION_MAJ << 3*8) | (EC_VERSION_MIN << 2*8) | (EC_VERSION_SERVICEPACK << 1*8) | (EC_VERSION_BUILD << 0*8))
#define EC_VERSION_TYPE              EC_VERSION_TYPE_PROTECTED

/*-VERSION STRINGS-----------------------------------------------------------*/
#define EC_FILEVERSIONSTR            "3.1.4.07 (Protected)\0"
#define EC_VERSION_TYPE_STR          "Protected"
#define EC_COPYRIGHT                 "Copyright acontis technologies GmbH @ 2022\0"

/*-LEGACY SUPPORT------------------------------------------------------------*/
/* the following definitions are deprecated and will be removed in V3.2 */
#define ATECAT_VERS_MAJ              EC_VERSION_MAJ
#define ATECAT_VERS_MIN              EC_VERSION_MIN
#define ATECAT_VERS_SERVICEPACK      EC_VERSION_SERVICEPACK
#define ATECAT_VERS_BUILD            EC_VERSION_BUILD
#define ATECAT_FILEVERSION           EC_VERSION_MAJ,EC_VERSION_MIN,EC_VERSION_SERVICEPACK,EC_VERSION_BUILD
#define ATECAT_VERSION               EC_VERSION
#define ATECAT_FILEVERSIONSTR        EC_FILEVERSIONSTR
#define ATECAT_PRODVERSION           EC_VERSION_MAJ,EC_VERSION_MIN,EC_VERSION_SERVICEPACK,EC_VERSION_BUILD
#define ATECAT_PRODVERSIONSTR        EC_FILEVERSIONSTR
#define ATECAT_COPYRIGHT             EC_COPYRIGHT

#endif /* INC_ECVERSION */

/*-END OF SOURCE FILE--------------------------------------------------------*/
