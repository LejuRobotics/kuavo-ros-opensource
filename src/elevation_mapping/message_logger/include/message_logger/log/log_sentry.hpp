#pragma once

#include <string>

#ifdef MELO_USE_SENTRY
#include <sentry.h>
#include <anymal_sentry_native/SentryGuard.hpp>
#include <anymal_sentry_native/init.hpp>
#include <anymal_sentry_native/package_metadata.hpp>
#endif

#include "message_logger/common/preprocessor_defines.hpp"
#include "message_logger/time/TimeStd.hpp"

#ifdef MELO_USE_SENTRY

namespace message_logger {

/*! Log a message to Sentry.
 *
 * @param message Message to log.
 * @param level Log level.
 */
void logMessageToSentry(const std::string& message, sentry_level_t level);

}  // namespace message_logger

#define MELO_SENTRY_LOG(level, message)                                             \
  {                                                                                 \
    ::anymal_sentry_native::ensureSentryGuardExists(PACKAGE_NAME, PACKAGE_VERSION); \
    if (::anymal_sentry_native::isInit()) {                                         \
      ::message_logger::logMessageToSentry((message), (level));                     \
    }                                                                               \
  }
#define MELO_SENTRY_LOG_ONCE(level, message) \
  {                                          \
    static bool isLogged{false};             \
    if (!isLogged) {                         \
      isLogged = true;                       \
      MELO_SENTRY_LOG(level, message)        \
    }                                        \
  }
#define MELO_SENTRY_LOG_THROTTLE(rate, level, message)                           \
  {                                                                              \
    static double lastLogged{0.0};                                               \
    ::message_logger::time::TimeStd now{::message_logger::time::TimeStd::now()}; \
    if (lastLogged + (rate) <= now.toSec()) {                                    \
      lastLogged = now.toSec();                                                  \
      MELO_SENTRY_LOG(level, message)                                            \
    }                                                                            \
  }

#else

#define MELO_SENTRY_LOG(level, ...)
#define MELO_SENTRY_LOG_ONCE(level, ...)
#define MELO_SENTRY_LOG_THROTTLE(rate, level, ...)

#endif
