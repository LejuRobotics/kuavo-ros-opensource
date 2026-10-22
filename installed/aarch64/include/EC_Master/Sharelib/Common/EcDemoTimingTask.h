/*-----------------------------------------------------------------------------
 * EcDemoTimingTask.h
 * Copyright                acontis technologies GmbH, Ravensburg, Germany
 * Response                 Timo Nussbaumer
 * Description              EtherCAT demo timing task
 *----------------------------------------------------------------------------*/

#pragma once

#ifndef EXECUTE_DEMOTIMINGTASK
#define EXECUTE_DEMOTIMINGTASK 1
#endif

#include "EcType.h"
#include "EcOs.h" //EC_T_UINT64

struct _T_EC_DEMO_APP_CONTEXT;
//enum _EC_T_LINKLAYER_TIMING;

class CDemoTimingEvent
{
public:
    CDemoTimingEvent();
    ~CDemoTimingEvent();
    EC_T_DWORD Create();
    EC_T_DWORD Set();
    EC_T_VOID  Delete();
    EC_T_VOID* GetRaw();

private:
    EC_T_VOID* m_pvTimingEvent; /* Timing event used by the timing task */
};

class CDemoTimingTask
{
public:
    CDemoTimingTask();
    explicit CDemoTimingTask(_T_EC_DEMO_APP_CONTEXT& rAppContext);
    virtual EC_T_DWORD CreateTimingEvent();
    virtual EC_T_DWORD SetTimingEvent();
    virtual EC_T_VOID  DeleteTimingEvent();
    virtual EC_T_VOID* GetRawTimingEvent();
    virtual EC_T_DWORD StartTimingTask(EC_T_INT nCycleTimeNsec);
    virtual EC_T_DWORD AdjustCycleTime(EC_T_INT nAdjustPermil);
    virtual EC_T_DWORD StopTimingTask();//~CDemoTimingTask() calls StopTimingTask() if overriden the destructor of the derived class must also call its StopTimingTask()
    virtual ~CDemoTimingTask();

protected:
    virtual EC_T_DWORD CreateThread();
    static  EC_T_VOID  TimingTaskWrapper(CDemoTimingTask* pDemoTimingTask);
    virtual EC_T_VOID  TimingTask();
    virtual EC_T_DWORD DeleteThread();

protected:
    _T_EC_DEMO_APP_CONTEXT* m_pAppContext;

public:
    EC_T_DWORD m_dwCpuIndex;              /* SMP systems: CPU index */
    EC_T_DWORD m_dwInstanceId;
    EC_T_INT   m_nCycleTimeNsec;          /* Cycle Time to use in nano seconds */
    EC_T_INT   m_nOriginalCycleTimeNsec;  /* Original cycle time in nano seconds */
    EC_T_BOOL  m_bShutdown;               /* EC_TRUE if thread shall shut down */
    EC_T_BOOL  m_bIsRunning;              /* EC_TRUE if the thread is running */
    EC_T_VOID* m_pvTimingThread;          /* Timing task handle the task of this handle sets the timing event */

public:
    static EC_T_DWORD GetTimeElapsedSinceCycleStart(EC_T_VOID* pvContext, EC_T_DWORD* pdwTimeElapsedSinceCycleStartInNsec); /* see also EC_PF_DC_GETTIMEELAPSEDSINCECYCLESTART, in this demo case pvContext is not used */
    static EC_T_DWORD AdjustCycleTime(EC_T_VOID* pvContext, EC_T_INT nAdjustPermil);                                        /* see also EC_PF_DC_ADJUSTCYCLETIME, in this demo case pContext corresponds to pTimingTaskData of the Start/StopTimingTask functions */
    static EC_T_DWORD GetHostTime(EC_T_VOID* pvContext, EC_T_UINT64* pnActualHostTimeInNsec);                               /* see also EC_PF_DC_GETHOSTTIME, in this demo case pContext corresponds to pTimingTaskData of the Start/StopTimingTask functions */

private:
    CDemoTimingEvent        m_oTimingEvent;
};

class CDemoLinkLayerTimingTask
    : public CDemoTimingTask
{
public:
    typedef CDemoTimingTask TBaseClass;

    explicit CDemoLinkLayerTimingTask(_T_EC_DEMO_APP_CONTEXT& rAppContext);
    virtual EC_T_DWORD StartTimingTask(EC_T_INT nCycleTimeNsec) EC_OVERRIDE;
    virtual EC_T_DWORD AdjustCycleTime(EC_T_INT nAdjustPermil) EC_OVERRIDE;
    virtual EC_T_DWORD StopTimingTask() EC_OVERRIDE;
    virtual ~CDemoLinkLayerTimingTask();

private:
    static void StartCycle(EC_T_VOID* pvStartCycleContext);

private:
    EC_T_WORD               m_nLinkLayerIndex;
};
