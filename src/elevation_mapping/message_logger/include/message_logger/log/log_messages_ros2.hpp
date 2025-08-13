/**********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024, ANYbotics
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Autonomous Systems Lab nor ETH Zurich
 *     nor the names of its contributors may be used to endorse or
 *     promote products derived from this software without specific
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
/*!
 * @file     log_messages_ros2.hpp
 * @author   ANYbotics
 * @date     2024
 * @brief
 */

#pragma once

#include <rclcpp/rclcpp.hpp>

#include "message_logger/log/log_messages_backend.hpp"

namespace message_logger {
namespace log {

class LoggerManager {
 public:
  static rclcpp::Logger& getLogger() {
    static rclcpp::Logger logger = rclcpp::get_logger("rclcpp");
    return logger;
  }

  static void setLogger(const rclcpp::Logger& logger) { getLogger() = logger; }
};

// Clock management
class LoggerClockManager {
 public:
  static rclcpp::Clock& getLoggerClock() {
    static rclcpp::Clock loggerClock(RCL_ROS_TIME);
    return loggerClock;
  }

  static void setClock(const rclcpp::Clock& clock) { getLoggerClock() = clock; }
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define MELO_LOG(level, ...)                                                                                                             \
  {                                                                                                                                      \
    auto& logger = message_logger::log::LoggerManager::getLogger();                                                                      \
    std::stringstream melo_stringstream;                                                                                                 \
    melo_stringstream << message_logger::log::parseMemberName(__PRETTY_FUNCTION__) << message_logger::log::getLogColor(level)            \
                      << message_logger::common::internal::meloStringFormat(__VA_ARGS__) << message_logger::log::getResetColor();        \
    switch (level) {                                                                                                                     \
      case message_logger::log::levels::Debug: {                                                                                         \
        RCLCPP_DEBUG(logger, "%s", melo_stringstream.str().c_str());                                                                     \
      } break;                                                                                                                           \
      case message_logger::log::levels::Info: {                                                                                          \
        RCLCPP_INFO(logger, "%s", melo_stringstream.str().c_str());                                                                      \
      } break;                                                                                                                           \
      case message_logger::log::levels::Warn: {                                                                                          \
        RCLCPP_WARN(logger, "%s", melo_stringstream.str().c_str());                                                                      \
      } break;                                                                                                                           \
      case message_logger::log::levels::Error: {                                                                                         \
        RCLCPP_ERROR(logger, "%s", melo_stringstream.str().c_str());                                                                     \
      } break;                                                                                                                           \
      case message_logger::log::levels::Fatal: {                                                                                         \
        RCLCPP_FATAL(logger, "%s", melo_stringstream.str().c_str());                                                                     \
        std::stringstream melo_assert_stringstream;                                                                                      \
        melo_assert_stringstream << message_logger::log::colorFatal << message_logger::common::internal::meloStringFormat(__VA_ARGS__)   \
                                 << message_logger::log::getResetColor();                                                                \
        message_logger::common::internal::meloThrowException<message_logger::log::melo_fatal>("[FATAL] ", __FUNCTION__, __FILE__,        \
                                                                                              __LINE__, melo_assert_stringstream.str()); \
      } break;                                                                                                                           \
      default: {                                                                                                                         \
        RCLCPP_INFO(logger, __VA_ARGS__);                                                                                                \
      } break;                                                                                                                           \
    }                                                                                                                                    \
  }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define MELO_LOG_STREAM(level, message)                                                                                                  \
  {                                                                                                                                      \
    auto& logger = message_logger::log::LoggerManager::getLogger();                                                                      \
    std::stringstream melo_stringstream;                                                                                                 \
    /* NOLINTNEXTLINE(bugprone-macro-parentheses) */                                                                                     \
    melo_stringstream << message_logger::log::parseMemberName(__PRETTY_FUNCTION__) << message_logger::log::getLogColor(level) << message \
                      << message_logger::log::getResetColor();                                                                           \
    switch (level) {                                                                                                                     \
      case message_logger::log::levels::Debug: {                                                                                         \
        RCLCPP_DEBUG_STREAM(logger, melo_stringstream.str());                                                                            \
      } break;                                                                                                                           \
      case message_logger::log::levels::Info: {                                                                                          \
        RCLCPP_INFO_STREAM(logger, melo_stringstream.str());                                                                             \
      } break;                                                                                                                           \
      case message_logger::log::levels::Warn: {                                                                                          \
        RCLCPP_WARN_STREAM(logger, melo_stringstream.str());                                                                             \
      } break;                                                                                                                           \
      case message_logger::log::levels::Error: {                                                                                         \
        RCLCPP_ERROR_STREAM(logger, melo_stringstream.str());                                                                            \
      } break;                                                                                                                           \
      case message_logger::log::levels::Fatal: {                                                                                         \
        RCLCPP_FATAL_STREAM(logger, melo_stringstream.str());                                                                            \
        std::stringstream melo_assert_stringstream;                                                                                      \
        /* NOLINTNEXTLINE(bugprone-macro-parentheses) */                                                                                 \
        melo_assert_stringstream << message_logger::log::colorFatal << message << message_logger::log::getResetColor();                  \
        message_logger::common::internal::meloThrowException<message_logger::log::melo_fatal>("[FATAL] ", __FUNCTION__, __FILE__,        \
                                                                                              __LINE__, melo_assert_stringstream.str()); \
      } break;                                                                                                                           \
      default: {                                                                                                                         \
        RCLCPP_INFO_STREAM(logger, message);                                                                                             \
      } break;                                                                                                                           \
    }                                                                                                                                    \
  }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define MELO_LOG_FP(level, ...) MELO_LOG(level, __VA_ARGS__)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define MELO_LOG_STREAM_FP(level, message) MELO_LOG_STREAM(level, message)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define MELO_LOG_ONCE(level, ...)                                                                                                        \
  {                                                                                                                                      \
    auto& logger = message_logger::log::LoggerManager::getLogger();                                                                      \
    std::stringstream melo_stringstream;                                                                                                 \
    melo_stringstream << message_logger::log::parseMemberName(__PRETTY_FUNCTION__) << message_logger::log::getLogColor(level)            \
                      << message_logger::common::internal::meloStringFormat(__VA_ARGS__) << message_logger::log::getResetColor();        \
    switch (level) {                                                                                                                     \
      case message_logger::log::levels::Debug: {                                                                                         \
        RCLCPP_DEBUG_ONCE(logger, "%s", melo_stringstream.str().c_str());                                                                \
      } break;                                                                                                                           \
      case message_logger::log::levels::Info: {                                                                                          \
        RCLCPP_INFO_ONCE(logger, "%s", melo_stringstream.str().c_str());                                                                 \
      } break;                                                                                                                           \
      case message_logger::log::levels::Warn: {                                                                                          \
        RCLCPP_WARN_ONCE(logger, "%s", melo_stringstream.str().c_str());                                                                 \
      } break;                                                                                                                           \
      case message_logger::log::levels::Error: {                                                                                         \
        RCLCPP_ERROR_ONCE(logger, "%s", melo_stringstream.str().c_str());                                                                \
      } break;                                                                                                                           \
      case message_logger::log::levels::Fatal: {                                                                                         \
        RCLCPP_FATAL_ONCE(logger, "%s", melo_stringstream.str().c_str());                                                                \
        std::stringstream melo_assert_stringstream;                                                                                      \
        melo_assert_stringstream << message_logger::log::colorFatal << message_logger::common::internal::meloStringFormat(__VA_ARGS__)   \
                                 << message_logger::log::getResetColor();                                                                \
        message_logger::common::internal::meloThrowException<message_logger::log::melo_fatal>("[FATAL] ", __FUNCTION__, __FILE__,        \
                                                                                              __LINE__, melo_assert_stringstream.str()); \
      } break;                                                                                                                           \
      default: {                                                                                                                         \
        RCLCPP_INFO_ONCE(logger, __VA_ARGS__);                                                                                           \
      } break;                                                                                                                           \
    }                                                                                                                                    \
  }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define MELO_LOG_STREAM_ONCE(level, message)                                                                                             \
  {                                                                                                                                      \
    auto& logger = message_logger::log::LoggerManager::getLogger();                                                                      \
    std::stringstream melo_stringstream;                                                                                                 \
    /* NOLINTNEXTLINE(bugprone-macro-parentheses) */                                                                                     \
    melo_stringstream << message_logger::log::parseMemberName(__PRETTY_FUNCTION__) << message_logger::log::getLogColor(level) << message \
                      << message_logger::log::getResetColor();                                                                           \
    switch (level) {                                                                                                                     \
      case message_logger::log::levels::Debug: {                                                                                         \
        RCLCPP_DEBUG_STREAM_ONCE(logger, melo_stringstream.str());                                                                       \
      } break;                                                                                                                           \
      case message_logger::log::levels::Info: {                                                                                          \
        RCLCPP_INFO_STREAM_ONCE(logger, melo_stringstream.str());                                                                        \
      } break;                                                                                                                           \
      case message_logger::log::levels::Warn: {                                                                                          \
        RCLCPP_WARN_STREAM_ONCE(logger, melo_stringstream.str());                                                                        \
      } break;                                                                                                                           \
      case message_logger::log::levels::Error: {                                                                                         \
        RCLCPP_ERROR_STREAM_ONCE(logger, melo_stringstream.str());                                                                       \
      } break;                                                                                                                           \
      case message_logger::log::levels::Fatal: {                                                                                         \
        RCLCPP_FATAL_STREAM_ONCE(logger, melo_stringstream.str());                                                                       \
        std::stringstream melo_assert_stringstream;                                                                                      \
        /* NOLINTNEXTLINE(bugprone-macro-parentheses) */                                                                                 \
        melo_assert_stringstream << message_logger::log::colorFatal << message << message_logger::log::getResetColor();                  \
        message_logger::common::internal::meloThrowException<message_logger::log::melo_fatal>("[FATAL] ", __FUNCTION__, __FILE__,        \
                                                                                              __LINE__, melo_assert_stringstream.str()); \
      } break;                                                                                                                           \
      default: {                                                                                                                         \
        RCLCPP_INFO_STREAM_ONCE(logger, message);                                                                                        \
      } break;                                                                                                                           \
    }                                                                                                                                    \
  }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define MELO_LOG_THROTTLE(rate, level, ...)                                                                                              \
  {                                                                                                                                      \
    auto& logger = message_logger::log::LoggerManager::getLogger();                                                                      \
    auto& clock = message_logger::log::LoggerClockManager::getLoggerClock();                                                             \
    std::stringstream melo_stringstream;                                                                                                 \
    melo_stringstream << message_logger::log::parseMemberName(__PRETTY_FUNCTION__) << message_logger::log::getLogColor(level)            \
                      << message_logger::common::internal::meloStringFormat(__VA_ARGS__) << message_logger::log::getResetColor();        \
    switch (level) {                                                                                                                     \
      case message_logger::log::levels::Debug: {                                                                                         \
        RCLCPP_DEBUG_THROTTLE(logger, clock, rate, "%s", melo_stringstream.str().c_str());                                               \
      } break;                                                                                                                           \
      case message_logger::log::levels::Info: {                                                                                          \
        RCLCPP_INFO_THROTTLE(logger, clock, rate, "%s", melo_stringstream.str().c_str());                                                \
      } break;                                                                                                                           \
      case message_logger::log::levels::Warn: {                                                                                          \
        RCLCPP_WARN_THROTTLE(logger, clock, rate, "%s", melo_stringstream.str().c_str());                                                \
      } break;                                                                                                                           \
      case message_logger::log::levels::Error: {                                                                                         \
        RCLCPP_ERROR_THROTTLE(logger, clock, rate, "%s", melo_stringstream.str().c_str());                                               \
      } break;                                                                                                                           \
      case message_logger::log::levels::Fatal: {                                                                                         \
        RCLCPP_FATAL_THROTTLE(logger, clock, rate, "%s", melo_stringstream.str().c_str());                                               \
        std::stringstream melo_assert_stringstream;                                                                                      \
        melo_assert_stringstream << message_logger::log::colorFatal << message_logger::common::internal::meloStringFormat(__VA_ARGS__)   \
                                 << message_logger::log::getResetColor();                                                                \
        message_logger::common::internal::meloThrowException<message_logger::log::melo_fatal>("[FATAL] ", __FUNCTION__, __FILE__,        \
                                                                                              __LINE__, melo_assert_stringstream.str()); \
      } break;                                                                                                                           \
      default: {                                                                                                                         \
        RCLCPP_INFO_THROTTLE(logger, clock, rate, __VA_ARGS__);                                                                          \
      } break;                                                                                                                           \
    }                                                                                                                                    \
  }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define MELO_LOG_THROTTLE_STREAM(rate, level, message)                                                                                   \
  {                                                                                                                                      \
    auto& logger = message_logger::log::LoggerManager::getLogger();                                                                      \
    auto& clock = message_logger::log::LoggerClockManager::getLoggerClock();                                                             \
    std::stringstream melo_stringstream;                                                                                                 \
    /* NOLINTNEXTLINE(bugprone-macro-parentheses) */                                                                                     \
    melo_stringstream << message_logger::log::parseMemberName(__PRETTY_FUNCTION__) << message_logger::log::getLogColor(level) << message \
                      << message_logger::log::getResetColor();                                                                           \
    switch (level) {                                                                                                                     \
      case message_logger::log::levels::Debug: {                                                                                         \
        RCLCPP_DEBUG_STREAM_THROTTLE(logger, clock, rate, melo_stringstream.str());                                                      \
      } break;                                                                                                                           \
      case message_logger::log::levels::Info: {                                                                                          \
        RCLCPP_INFO_STREAM_THROTTLE(logger, clock, rate, melo_stringstream.str());                                                       \
      } break;                                                                                                                           \
      case message_logger::log::levels::Warn: {                                                                                          \
        RCLCPP_WARN_STREAM_THROTTLE(logger, clock, rate, melo_stringstream.str());                                                       \
      } break;                                                                                                                           \
      case message_logger::log::levels::Error: {                                                                                         \
        RCLCPP_ERROR_STREAM_THROTTLE(logger, clock, rate, melo_stringstream.str());                                                      \
      } break;                                                                                                                           \
      case message_logger::log::levels::Fatal: {                                                                                         \
        RCLCPP_FATAL_STREAM_THROTTLE(logger, clock, rate, melo_stringstream.str());                                                      \
        std::stringstream melo_assert_stringstream;                                                                                      \
        /* NOLINTNEXTLINE(bugprone-macro-parentheses) */                                                                                 \
        melo_assert_stringstream << message_logger::log::colorFatal << message << message_logger::log::getResetColor();                  \
        message_logger::common::internal::meloThrowException<message_logger::log::melo_fatal>("[FATAL] ", __FUNCTION__, __FILE__,        \
                                                                                              __LINE__, melo_assert_stringstream.str()); \
      } break;                                                                                                                           \
      default: {                                                                                                                         \
        RCLCPP_INFO_STREAM_THROTTLE(logger, clock, rate, message);                                                                       \
      } break;                                                                                                                           \
    }                                                                                                                                    \
  }

}  // namespace log
}  // namespace message_logger
