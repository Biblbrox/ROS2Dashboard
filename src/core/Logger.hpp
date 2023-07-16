#pragma once

#include <spdlog/sinks/daily_file_sink.h>
#include <spdlog/sinks/stdout_sinks.h>
#include <spdlog/sinks/systemd_sink.h>
#include <spdlog/spdlog.h>
#include <string>
#include <string_view>

namespace ros2monitor {
enum class Category { debug,
                      info,
                      critical,
                      err,
                      off,
                      trace,
                      warn };

class Logger {
public:
    static void init(Category level, const std::string &fileName);

    static void setLevel(Category category);

    template<typename... Args>
    static void info(std::string_view fmt_str, Args &&...args)
    {
        m_logger->template info(fmt_str, std::forward<Args>(args)...);
    }

    template<typename... Args>
    static void error(std::string_view fmt_str, Args &&...args)
    {
        m_logger->template error(fmt_str, std::forward<Args>(args)...);
    }

    template<typename... Args>
    static void warn(std::string_view fmt_str, Args &&...args)
    {
        m_logger->template warn(fmt_str, std::forward<Args>(args)...);
    }

    template<typename... Args>
    static void debug(std::string_view fmt_str, Args &&...args)
    {
        m_logger->template debug(fmt_str, std::forward<Args>(args)...);
    }

public:
    static std::shared_ptr<spdlog::logger> m_logger;
};
};// namespace logger
