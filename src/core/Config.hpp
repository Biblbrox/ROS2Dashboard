#pragma once

#define TOML_EXCEPTIONS 0// Disable exceptions
#include <toml++/toml.h>

namespace ros2monitor {

class Config {
    /**
     * @brief The Config class with Toml file format
     */
public:
    explicit Config(const std::string &path);

    /**
            * @brief Get the value of the key
            * @param key
            * @return value
            */
    template<typename T>
    T get(const std::string &key) const
    {
        return m_data[key].value<T>();
    }

    template<class T>
    T getByIdx(const size_t &idx) const
    {
        auto it = m_data.begin();
        std::advance(it, idx);
        return it->second.value<T>();
    }

    [[nodiscard]] size_t size() const
    {
        return m_data.size();
    }

private:
    toml::table m_data;
};

}// namespace ros2monitor
