#include "message_logger/log/log_sentry.hpp"

#ifdef MELO_USE_SENTRY

#include <ros/this_node.h>

#include <sentry.h>

namespace message_logger {

std::string convertLevelToString(const sentry_level_t level) {
  switch (level) {
    case SENTRY_LEVEL_DEBUG:
      return "debug";
    case SENTRY_LEVEL_INFO:
      return "info";
    case SENTRY_LEVEL_WARNING:
      return "warning";
    case SENTRY_LEVEL_ERROR:
      return "error";
    case SENTRY_LEVEL_FATAL:
      return "fatal";
    default:
      return "unknown";
  }
}

void logMessageToSentry(const std::string& message, const sentry_level_t level) {
  const std::string enhancedMessage{"[" + ros::this_node::getName() + "] " + message};
  if (level == SENTRY_LEVEL_ERROR || level == SENTRY_LEVEL_FATAL) {
    sentry_value_t event{sentry_value_new_message_event(level, "message_logger", enhancedMessage.c_str())};
    sentry_value_set_stacktrace(event, nullptr, 0);
    sentry_capture_event(event);
  }
  // Add errors/fatals also a breadcrumbs for future errors/fatals.
  if (::anymal_sentry_native::getCachedMinBreadcrumbLevel() <= level) {
    sentry_value_t crumb{sentry_value_new_breadcrumb("default", enhancedMessage.c_str())};
    sentry_value_set_by_key(crumb, "level", sentry_value_new_string(convertLevelToString(level).c_str()));
    // To make it obvious in the UI we set the category to "breadcrumb".
    sentry_value_set_by_key(crumb, "category", sentry_value_new_string("breadcrumb"));
    sentry_add_breadcrumb(crumb);
  }
}

}  // namespace message_logger

#endif
