#include "Config.hpp"
#include "Logger.hpp"

namespace ros2monitor {

Config::Config(const std::string &path)
{
    toml::parse_result result = toml::parse_file(path);
    if (!result) {
        Logger::error(fmt::format("Parsing failed. Error: {}", result.error().description()));
        return;
    }

    m_data = std::move(result).table();
}

}// namespace ros2monitor