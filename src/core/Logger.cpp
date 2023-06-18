#include <iostream>
#include <spdlog/common.h>
#include <spdlog/logger.h>
#include <spdlog/sinks/daily_file_sink.h>
#include <spdlog/sinks/stdout_sinks.h>
#include <spdlog/sinks/systemd_sink.h>
#include <spdlog/spdlog.h>
#include <vector>

#include "Logger.hpp"

namespace ros2monitor {
    std::shared_ptr<spdlog::logger> ros2monitor::Logger::m_logger;

    void Logger::init(Category level, const std::string &fileName)
    {
        std::vector<spdlog::sink_ptr> sinks;
        if (level == Category::debug)
            sinks.push_back(std::make_shared<spdlog::sinks::stdout_sink_st>());
        sinks.push_back(std::make_shared<spdlog::sinks::daily_file_sink_st>(fileName, 23, 59));
        sinks.push_back(std::make_shared<spdlog::sinks::systemd_sink_st>());
        m_logger = std::make_shared<spdlog::logger>("logger", begin(sinks), end(sinks));
        m_logger->set_level(spdlog::level::debug);
        spdlog::register_logger(m_logger);
        std::cout << "Logger initialized\n";
    }

    void Logger::setLevel(Category category)
    {
        switch (category) {
            case Category::debug:
                spdlog::set_level(spdlog::level::level_enum::debug);
                break;
            case Category::info:
                spdlog::set_level(spdlog::level::level_enum::info);
                break;
            case Category::critical:
                spdlog::set_level(spdlog::level::level_enum::critical);
                break;
            case Category::err:
                spdlog::set_level(spdlog::level::level_enum::err);
                break;
            case Category::off:
                spdlog::set_level(spdlog::level::level_enum::off);
                break;
            case Category::trace:
                spdlog::set_level(spdlog::level::level_enum::trace);
                break;
            case Category::warn:
                spdlog::set_level(spdlog::level::level_enum::warn);
                break;
        }
    }

}