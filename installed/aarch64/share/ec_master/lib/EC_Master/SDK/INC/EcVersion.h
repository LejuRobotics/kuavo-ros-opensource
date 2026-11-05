/** ---------------------------------------------------------------------------
 * \file
 * \brief       EC-Master version information
 * \copyright   acontis technologies GmbH, D-88212 Weingarten, Germany
 *---------------------------------------------------------------------------*/

#ifndef INC_ECVERSION
#define INC_ECVERSION 1

#define EC_VERSION_MAKE(a,b,c,d)     (    ((a)<<24)|((b)<<16)|((c)<<8)|(d))
#define EC_SIGNATURE_MAKE(p,a,b,c,d) ((p)|((a)<<16)|((b)<<12)|((c)<<8)|(d))

/*-DEFINES-------------------------------------------------------------------*/
/* EC_VERSION_TYPE_... */
/** \defgroup EC_VERSION_TYPES EC-Master Version Types
@{*/
#define EC_VERSION_TYPE_UNDEFINED    0  /**<  EC-Master Type Undefined */
#define EC_VERSION_TYPE_UNRESTRICTED 1  /**<  EC-Master Type Unrestricted */
#define EC_VERSION_TYPE_PROTECTED    2  /**<  EC-Master Type Protected */
#define EC_VERSION_TYPE_DONGLED      3  /**<  EC-Master Type Dongled */
#define EC_VERSION_TYPE_EVAL         4  /**<  EC-Master Type Eval */
/**@}*/ /* EC_VERSION_TYPE */

/*-VERSION INFORMATION-------------------------------------------------------*/
#define EC_VERSION_MAJ               3   /* major version */
#define EC_VERSION_MIN               3   /* minor version */
#define EC_VERSION_SERVICEPACK       3   /* service pack */
#define EC_VERSION_BUILD             1   /* build number */
#define EC_VERSION                   EC_VERSION_MAKE(EC_VERSION_MAJ, EC_VERSION_MIN, EC_VERSION_SERVICEPACK, EC_VERSION_BUILD)

/*-VERSION STRINGS-----------------------------------------------------------*/
#define EC_VERSION_STRINGIZE(x) #x
#define EC_VERSION_QUOTE(x) EC_VERSION_STRINGIZE(x)
#if EC_VERSION_BUILD < 10
#define EC_VERSION_BUILD_PREFIX "0"
#else
#define EC_VERSION_BUILD_PREFIX ""
#endif
#define EC_VERSION_NUM_STR EC_VERSION_QUOTE(EC_VERSION_MAJ) "." EC_VERSION_QUOTE(EC_VERSION_MIN) "." EC_VERSION_QUOTE(EC_VERSION_SERVICEPACK) "." EC_VERSION_BUILD_PREFIX EC_VERSION_QUOTE(EC_VERSION_BUILD)

#define EC_VERSION_TYPE          EC_VERSION_TYPE_PROTECTED
#define EC_VERSION_TYPE_STR      "Protected"
#if (defined DONGLED_VERSION) || (defined PROTECTED_VERSION) || (defined EVAL_VERSION)
#error Do not use the build configurations Dongled, Protected or Eval as this code has been removed from the SDK Source packages! Use Release or Debug instead!
#endif

#define EC_FILEVERSIONSTR EC_VERSION_NUM_STR " (" EC_VERSION_TYPE_STR ")\0"

#define EC_COPYRIGHT "Copyright acontis technologies GmbH @ 2026\0"

#define EC_VERSION_SINCE(a,b,c,d) (EC_VERSION >= EC_VERSION_MAKE(a,b,c,d))
#define EC_VERSION_WITHIN_2(a,b) ((a <= EC_VERSION) && (EC_VERSION <= b))
#define EC_VERSION_WITHIN(vlmaj,vlmin,vlsp,vlb,spacer,vumaj,vumin,vusp,vub) EC_VERSION_WITHIN_2(EC_VERSION_MAKE(vlmaj,vlmin,vlsp,vlb), EC_VERSION_MAKE(vumaj,vumin,vusp,vub))

#endif /* INC_ECVERSION */

/*-END OF SOURCE FILE--------------------------------------------------------*/
