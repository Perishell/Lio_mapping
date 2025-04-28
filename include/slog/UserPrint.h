//
// Created by shulei.sheng on 2021/8/30.
//
#ifndef UGV_USERPRINT_H
#define UGV_USERPRINT_H
#include <cstdio>
#include <cstdarg>
#include "spdlog/spdlog.h"
#include "slog/LogMgmt.h"
#include <ros/ros.h>
static void PrintMsg(const char *format, ...)
{
    char buffer[1024] = {0};
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, 1024, format, args);
    va_end(args);
    printf("%s", buffer);
}
static std::string SpdPrintMsg(const char *format, ...)
{
    char buffer[1024] = {0};
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, 1024, format, args);
    va_end(args);
    return std::string(buffer);
}

extern void PrintMsg(const char *format, ...);
extern std::string SpdPrintMsg(const char *format, ...);

#define GET_FILE_NAME(x) std::strrchr(x,'/')? std::strrchr(x,'/')+1:x
#define USER_PRINT_BASE(format, args...) PrintMsg(format, ##args)

#define LTrace(...)  LogMgmt::GetInstance()->GetLogger()->trace(__VA_ARGS__)
#define LDebug(...)  LogMgmt::GetInstance()->GetLogger()->debug(__VA_ARGS__)
#define LInfo(...)  LogMgmt::GetInstance()->GetLogger()->info(__VA_ARGS__)
#define LWarn(...) LogMgmt::GetInstance()->GetLogger()->warn(__VA_ARGS__)
#define LError(...)  LogMgmt::GetInstance()->GetLogger()->error(__VA_ARGS__);
#define LCritical(...)  LogMgmt::GetInstance()->GetLogger()->critical(__VA_ARGS__)

#define USER_PRINT_Debug(format, args...) \
{\
    USER_PRINT_BASE("[Debug]\033[0;32;40m<%s, %s,%d>\033[0m" format "\n", GET_FILE_NAME(__FILE__), __FUNCTION__, __LINE__, ##args); \
}
#define USER_PRINT_0(format, args...) \
{\
    USER_PRINT_BASE("\033[0;32;40m<%s, %s,%d>\033[0m" format "\n", GET_FILE_NAME(__FILE__), __FUNCTION__, __LINE__, ##args); \
}
#define USER_PRINT_1(format, args...) \
{\
    USER_PRINT_BASE("\033[0;33;40m<%s, %s,%d>" format "\033[0m\n", GET_FILE_NAME(__FILE__), __FUNCTION__, __LINE__, ##args); \
}
#define USER_PRINT_2(format, args...) \
{\
    USER_PRINT_BASE("\033[0;31;40m<%s, %s,%d>" format "\033[0m\n", GET_FILE_NAME(__FILE__), __FUNCTION__, __LINE__, ##args); \
}
#define SPD_TRACE(format, args...)\
{\
    LTrace(SpdPrintMsg("<%s, %s,%d>" format, GET_FILE_NAME(__FILE__), __FUNCTION__, __LINE__, ##args).c_str()); \
}
#define SPD_DEBUG(format, args...)\
{\
        std::string tempMsg = SpdPrintMsg("<%s, %s,%d>" format, GET_FILE_NAME(__FILE__), __FUNCTION__, __LINE__, ##args);                            \
        LDebug(tempMsg.c_str());      \
        spdlog::get("logger")->debug(tempMsg.c_str());                          \
}
#define SPD_INFO(format, args...)\
{                                \
    std::string tempMsg = SpdPrintMsg("<%s, %s,%d>" format, GET_FILE_NAME(__FILE__), __FUNCTION__, __LINE__, ##args);                            \
    LInfo(tempMsg.c_str());          \
    spdlog::get("logger")->info(tempMsg.c_str());          \
}
#define SPD_WARN(format, args...)\
{                                \
    std::string tempMsg = SpdPrintMsg("<%s, %s,%d>" format, GET_FILE_NAME(__FILE__), __FUNCTION__, __LINE__, ##args);                            \
    LWarn(tempMsg.c_str());          \
    spdlog::get("logger")->warn(tempMsg.c_str());                             \
}
#define SPD_ERROR(format, args...)\
{\
    std::string tempMsg = SpdPrintMsg("<%s, %s,%d>" format, GET_FILE_NAME(__FILE__), __FUNCTION__, __LINE__, ##args);                            \
    LError(tempMsg.c_str()); \
    spdlog::get("logger")->error(tempMsg.c_str());                              \
}

#define PrintTrace(format, args...) {\
    if(g_debug_mode) \
    {                                \
        if(g_use_spdlog)             \
            SPD_TRACE(format, ##args)               \
        else\
            USER_PRINT_Debug(format, ##args)\
    }\
}
#define PrintDebug(format, args...) {\
    SPD_DEBUG(format, ##args);\
}
#define PrintInfo(format, args...) {\
    SPD_INFO(format, ##args);\
}
#define PrintWarn(format, args...)  {\
    SPD_WARN(format, ##args);\
}
#define PrintError(format, args...) {\
    SPD_ERROR(format, ##args);\
}
#endif //UGV_USERPRINT_H
