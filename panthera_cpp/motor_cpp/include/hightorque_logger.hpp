#ifndef HIGHTORQUE_LOGGER_HPP
#define HIGHTORQUE_LOGGER_HPP

#include <atomic>
#include <cstdio>

namespace hightorque_robot
{
enum class LogLevel
{
    Error = 0,
    Warn = 1,
    Info = 2,
    Debug = 3,
    Off = 4,
};

inline std::atomic<int>& log_level_storage()
{
    static std::atomic<int> level{static_cast<int>(LogLevel::Info)};
    return level;
}

inline void set_log_level(LogLevel level)
{
    log_level_storage().store(static_cast<int>(level));
}

inline LogLevel get_log_level()
{
    return static_cast<LogLevel>(log_level_storage().load());
}

inline bool should_log(LogLevel level)
{
    const int configured = log_level_storage().load();
    return configured != static_cast<int>(LogLevel::Off) && static_cast<int>(level) <= configured;
}
} // namespace hightorque_robot

#define HT_LOG_INFO(format, ...) \
    do { if (::hightorque_robot::should_log(::hightorque_robot::LogLevel::Info)) std::printf(format "\n", ##__VA_ARGS__); } while (0)
#define HT_LOG_ERROR(format, ...) \
    do { if (::hightorque_robot::should_log(::hightorque_robot::LogLevel::Error)) std::fprintf(stderr, "\033[1;31m" format "\033[0m\n", ##__VA_ARGS__); } while (0)
#define HT_LOG_DEBUG(format, ...) \
    do { if (::hightorque_robot::should_log(::hightorque_robot::LogLevel::Debug)) std::printf(format "\n", ##__VA_ARGS__); } while (0)

#endif
