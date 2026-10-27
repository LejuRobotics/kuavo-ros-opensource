#pragma once

// Provide default definitions so static analyzers (e.g. cpplint) can see macros
// even when they are injected only via build system.
//
// Runtime contract is still fail-fast: empty string will be rejected by
// SolverTools::RequireConfigDir (no fallback).

#ifndef KUAVO_ANKLE_CONFIG_DIR
#define KUAVO_ANKLE_CONFIG_DIR ""
#endif

#ifndef KUAVO_ARM_CONFIG_DIR
#define KUAVO_ARM_CONFIG_DIR ""
#endif

#ifndef KUAVO_WAIST_CONFIG_DIR
#define KUAVO_WAIST_CONFIG_DIR ""
#endif

