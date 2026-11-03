#pragma once

#include <cstdint>
#include <limits>

// The robot integration keeps microseconds internally. Only the EC-Master 3.3
// API boundary uses nanoseconds. Reject zero and conversions that overflow DWORD.
inline bool EcBusCycleUsecToNsec(std::uint32_t cycleUsec, std::uint32_t& cycleNsec)
{
    cycleNsec = 0;
    if (cycleUsec == 0 || cycleUsec > std::numeric_limits<std::uint32_t>::max() / 1000U)
    {
        return false;
    }
    cycleNsec = cycleUsec * 1000U;
    return true;
}
