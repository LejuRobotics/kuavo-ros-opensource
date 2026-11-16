#pragma once
#include "sg100_types.h"

// Same units as the deployment SDK: 0.6 must encode as 0.6, never 6.0.
inline uint16_t encodeSg100OutputLimit(float value) {
    return SG100_Quant::encodeOutLim(value);
}
