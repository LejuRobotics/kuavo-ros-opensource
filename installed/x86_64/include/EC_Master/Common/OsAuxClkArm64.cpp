#include "EcOs.h"

EC_T_DWORD OsAuxClkInit(EC_T_DWORD dwCpuIndex, EC_T_DWORD dwFrequencyHz, EC_T_VOID* pvOsEvent)
{
    (void)dwCpuIndex;
    (void)dwFrequencyHz;
    (void)pvOsEvent;
    return EC_E_NOERROR;
}

EC_T_DWORD OsAuxClkDeinit(EC_T_VOID)
{
    return EC_E_NOERROR;
}
