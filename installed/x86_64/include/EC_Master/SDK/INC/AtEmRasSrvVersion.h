/*-----------------------------------------------------------------------------
 * AtEmRasSrvVersion.h
 * Copyright            acontis technologies GmbH, D-88212 Ravensburg, Germany
 * Description          EC-Master RAS Server version information
 *---------------------------------------------------------------------------*/

#ifndef INC_ATEMRASSRVVERSION
#define INC_ATEMRASSRVVERSION 1

/*-DEFINES-------------------------------------------------------------------*/
#define ATEMRASSRV_VERS_MAJ             3   /* major version */
#define ATEMRASSRV_VERS_MIN             1   /* minor version */
#define ATEMRASSRV_VERS_SERVICEPACK     4   /* service pack */
#define ATEMRASSRV_VERS_BUILD          7   /* build number */

#define ATEMRASSRV_VERSION (                   \
      (ATEMRASSRV_VERS_MAJ   << 3*8)           \
    | (ATEMRASSRV_VERS_MIN   << 2*8)           \
    | (ATEMRASSRV_VERS_SERVICEPACK << 1*8)     \
    | (ATEMRASSRV_VERS_BUILD << 0*8)           \
                        )

#define ATEMRASSRV_FILEVERSION     ATEMRASSRV_VERS_MAJ,ATEMRASSRV_VERS_MIN,ATEMRASSRV_VERS_SERVICEPACK,ATEMRASSRV_VERS_BUILD
#define ATEMRASSRV_FILEVERSIONSTR  "3.1.4.07\0"

#endif /* INC_ATEMRASSRVVERSION */

/*-END OF SOURCE FILE--------------------------------------------------------*/

