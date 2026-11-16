/*-----------------------------------------------------------------------------
 * EcDemoPlatform.h
 * Copyright                acontis technologies GmbH, Ravensburg, Germany
 * Response                 Holger Oelhaf
 * Description              Platform specific settings for EC demo
 *---------------------------------------------------------------------------*/

#ifndef INC_ECDEMOPLATFORM_H
#define INC_ECDEMOPLATFORM_H 1

/*-DEFINES-------------------------------------------------------------------*/
/* set all threads to highest possible priority to avoid errors */
#define TIMER_THREAD_PRIO           ((EC_T_DWORD)99)   /* EtherCAT master timer task (tEcTimingTask) */
#define JOBS_THREAD_PRIO            ((EC_T_DWORD)98)   /* EtherCAT master job thread priority (EcMasterJobTask) */
#define RECV_THREAD_PRIO            ((EC_T_DWORD)97)   /* EtherCAT master packet receive thread priority (tLOsaL_IST) */
#define LOG_THREAD_PRIO             ((EC_T_DWORD)29)   /* EtherCAT message logging thread priority (tAtEmLog) */
#define MAIN_THREAD_PRIO            ((EC_T_DWORD)39)   /* Main thread */
#define REMOTE_RECV_THREAD_PRIO     ((EC_T_DWORD)39)

#define TIMER_THREAD_STACKSIZE       0x1000
#if ((EC_ARCH == EC_ARCH_X64) || (EC_ARCH == EC_ARCH_ARM64))
#define JOBS_THREAD_STACKSIZE        0x8000
#define LOG_THREAD_STACKSIZE         0x8000
#else
#define JOBS_THREAD_STACKSIZE        0x4000
#define LOG_THREAD_STACKSIZE         0x4000
#endif

#define AUXCLOCK_SUPPORTED

#endif /* INC_ECDEMOPLATFORM_H */

/*-END OF SOURCE FILE--------------------------------------------------------*/
