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
    m_path = path;
}

void Config::save(const std::string &path)
{
    std::ofstream file(path);
    file << m_data;
}
std::string Config::path() const
{
    return m_path;
}

}// namespace ros2monitor