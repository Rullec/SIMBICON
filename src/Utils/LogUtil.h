#pragma once
#include "spdlog/sinks/stdout_color_sinks.h"
#include <memory>
#include <spdlog/fmt/ostr.h>
#include <spdlog/spdlog.h>
typedef std::shared_ptr<spdlog::logger> tLogger;
typedef spdlog::level::level_enum eLogLevel;

class cLogUtil
{
public:
    static void SetLoggingLevel(const std::string &level_name);
    static tLogger CreateLogger(const std::string &loggername);
    static void DropLogger(const std::string &loggername);
    static tLogger GetLogger(const std::string &loggername);
    static void Printf(const tLogger &logger, eLogLevel level, const char *fmt,
                       va_list args);

    // const static tLogger mGlobalLogger;
    inline const static tLogger mGlobalLogger =
        cLogUtil::CreateLogger("Global");

private:
    inline const static size_t buf_size = 1000;
    inline static char buf[buf_size];
    static int GetLevelEnumFromString(const std::string &level_name);
};

#if defined(_WIN32)
#define CON_UNREACHABLE __assume(0);
#else
#define CON_UNREACHABLE __builtin_unreachable();
#endif

#define __FILENAME__                                                           \
    (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#define SPD_AUGMENTED_LOG(X, ...)                                              \
    cLogUtil::mGlobalLogger->X(                                                \
        fmt::format("[{}:{}@{}] ", __FILENAME__, __FUNCTION__, __LINE__) +     \
        fmt::format(__VA_ARGS__))

#define CON_OUTPUT(...) cLogUtil::mGlobalLogger->info(fmt::format(__VA_ARGS__))

#define CON_TRACE(...) SPD_AUGMENTED_LOG(trace, __VA_ARGS__)
#define CON_DEBUG(...) SPD_AUGMENTED_LOG(debug, __VA_ARGS__)
#define CON_INFO(...) SPD_AUGMENTED_LOG(info, __VA_ARGS__)
#define CON_WARN(...) SPD_AUGMENTED_LOG(warn, __VA_ARGS__)
#define CON_ERROR(...)                                                         \
    {                                                                          \
        SPD_AUGMENTED_LOG(error, __VA_ARGS__);                                 \
        CON_UNREACHABLE;                                                       \
    }

#define CON_ASSERT_INFO(x, ...)                                                \
    {                                                                          \
        bool ___ret___ = static_cast<bool>(x);                                 \
        if (!___ret___)                                                        \
        {                                                                      \
            CON_ERROR(__VA_ARGS__);                                            \
        }                                                                      \
    }

#define CON_ASSERT(x) CON_ASSERT_INFO((x), #x)
class Vector3d;
class Quaternion;

std::ostream &operator<<(std::ostream &os, const Vector3d &vec);
std::ostream &operator<<(std::ostream &os, const Quaternion &vec);
