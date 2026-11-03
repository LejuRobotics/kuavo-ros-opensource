#ifndef _MOTOR_SDK_LOG_H_
#define _MOTOR_SDK_LOG_H_

#define MOTOR_PRINT_LOG
#ifdef MOTOR_PRINT_LOG

#include <stdio.h>
#include <stdarg.h>
#include <unistd.h>
#include <string.h>
static void __motor_log(const char*tag, const char* format, ...) 
{
    va_list args_size;
    va_start(args_size, format);
    int size = vsnprintf(nullptr, 0, format, args_size);
    va_end(args_size);

    va_list args;
    va_start(args, format);
    char* buffer = new char[size + 1]();
    vsprintf(buffer, format, args);
    va_end(args);

    printf("RUIWO [%s] %s\n", tag, buffer);

    if (nullptr != buffer) {
        delete[] buffer;
        buffer = nullptr;
    }
}

#define RWLOG_I(...) __motor_log("INFO", __VA_ARGS__)
#define RWLOG_D(...) __motor_log("DEBUG", __VA_ARGS__)
#define RWLOG_E(...) __motor_log("ERROR", __VA_ARGS__)
#define RWLOG_T(...) __motor_log("TRACE",  __VA_ARGS__)
#define RWLOG_W(...) __motor_log("WARN",  __VA_ARGS__)

// Colored log macros
#define RWLOG_SUCCESS(format, ...) RWLOG_I("\033[32m" format "\033[0m", ##__VA_ARGS__)
#define RWLOG_WARNING(format, ...) RWLOG_W("\033[33m" format "\033[0m", ##__VA_ARGS__)
#define RWLOG_FAILURE(format, ...) RWLOG_E("\033[31m" format "\033[0m", ##__VA_ARGS__)

#else

#define RWLOG_I(...)
#define RWLOG_D(...)
#define RWLOG_E(...)
#define RWLOG_T(...)
#define RWLOG_W(...) 
#define RWLOG_SUCCESS(...)
#define RWLOG_WARNING(...)
#define RWLOG_FAILURE(...)

#endif // MOTOR_PRINT_LOG

#endif // _MOTOR_SDK_LOG_H_