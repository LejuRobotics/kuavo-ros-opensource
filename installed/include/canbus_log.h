#ifndef _CANBUS_LOG_H_
#define _CANBUS_LOG_H_

#define CANBUS_PRINT_LOG /* 是否打印日志 */

#ifdef CANBUS_PRINT_LOG

#include <stdio.h>
#include <stdarg.h>
#include <unistd.h>
#include <string.h>
static void __canbus_log(const char*tag, const char* format, ...) 
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

    printf("CANBUS_SDK [%s] %s\n", tag, buffer);

    if (nullptr != buffer) {
        delete[] buffer;
        buffer = nullptr;
    }
}

#define LOG_I(...) __canbus_log("INFO", __VA_ARGS__)
#define LOG_D(...) __canbus_log("DEBUG", __VA_ARGS__)
#define LOG_E(...) __canbus_log("ERROR", __VA_ARGS__)
#define LOG_T(...) __canbus_log("TRACE",  __VA_ARGS__)
#define LOG_W(...) __canbus_log("WARN",  __VA_ARGS__)
// Colored log macros
#define LOG_SUCCESS(format, ...) LOG_I("\033[32m" format "\033[0m", ##__VA_ARGS__)
#define LOG_WARNING(format, ...) LOG_W("\033[33m" format "\033[0m", ##__VA_ARGS__)
#define LOG_FAILURE(format, ...) LOG_E("\033[31m" format "\033[0m", ##__VA_ARGS__)
#else

#define LOG_I(...) 
#define LOG_D(...) 
#define LOG_E(...) 
#define LOG_T(...) 
#define LOG_W(...) 
#define LOG_SUCCESS(...) 
#define LOG_WARNING(...) 
#define LOG_FAILURE(...) 

#endif // CANBUS_PRINT_LOG

#endif // _CANBUS_LOG_H_