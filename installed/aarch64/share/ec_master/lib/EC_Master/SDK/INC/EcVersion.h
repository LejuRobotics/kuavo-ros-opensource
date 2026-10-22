/*-----------------------------------------------------------------------------
 * EcVersion.h
 * Copyright            acontis technologies GmbH, D-88212 Ravensburg, Germany
 * Description          EC-Master version information
 *---------------------------------------------------------------------------*/

#ifndef INC_ECVERSION
#define INC_ECVERSION 1

#define EC_VERSION_MAKE(a,b,c,d) (((a)<<24)+((b)<<16)+((c)<<8)+d)

/*-DEFINES-------------------------------------------------------------------*/
/* EC_VERSION_TYPE_... */
#define EC_VERSION_TYPE_UNDEFINED    0
#define EC_VERSION_TYPE_UNRESTRICTED 1
#define EC_VERSION_TYPE_PROTECTED    2
#define EC_VERSION_TYPE_DONGLED      3
#define EC_VERSION_TYPE_EVAL         4

/*-VERSION INFORMATION-------------------------------------------------------*/
#define EC_VERSION_MAJ               3   /* major version */
#define EC_VERSION_MIN               2   /* minor version */
#define EC_VERSION_SERVICEPACK       1   /* service pack */
#define EC_VERSION_BUILD             99   /* build number */
#define EC_VERSION                   EC_VERSION_MAKE(EC_VERSION_MAJ, EC_VERSION_MIN, EC_VERSION_SERVICEPACK, EC_VERSION_BUILD)

#define EC_VERSION_TYPE              EC_VERSION_TYPE_PROTECTED

/*-VERSION STRINGS-----------------------------------------------------------*/
#ifndef EC_VERSION_NUM_STR 
#define EC_VERSION_NUM_STR "3.2.1.99"
#endif

#define EC_FILEVERSIONSTR        "3.2.1.99 (Protected)\0"
#define EC_VERSION_TYPE_STR      "Protected"

#define EC_COPYRIGHT "Copyright acontis technologies GmbH @ 2024\0"

#define EC_VERSION_SINCE(a,b,c,d) (EC_VERSION >= EC_VERSION_MAKE(a,b,c,d))
#define EC_VERSION_WITHIN_2(a,b) ((a <= EC_VERSION) && (EC_VERSION <= b))
#define EC_VERSION_WITHIN(vlmaj,vlmin,vlsp,vlb,spacer,vumaj,vumin,vusp,vub) EC_VERSION_WITHIN_2(EC_VERSION_MAKE(vlmaj,vlmin,vlsp,vlb), EC_VERSION_MAKE(vumaj,vumin,vusp,vub))

#endif /* INC_ECVERSION */

/*-END OF SOURCE FILE--------------------------------------------------------*/
