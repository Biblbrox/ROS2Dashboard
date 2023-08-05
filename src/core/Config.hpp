#pragma once

#define TOML_EXCEPTIONS 0// Disable exceptions
#include <toml++/toml.h>

namespace ros2monitor {

class Config {
    /**
     * @brief The Config class with Toml file format
     */
public:
    /**
     * Iterators for Config class
     */
    using iterator = toml::table::iterator;
    using const_iterator = toml::table::const_iterator;

    [[nodiscard]] bool contains(const std::string &key) const
    {
        // Check if key is nested
        if (key.find('.') != std::string::npos) {
            std::string group = key.substr(0, key.find('.'));
            std::string name = key.substr(key.find('.') + 1);

            if (m_data.find(group) == m_data.end()) {
                return false;
            }

            return true;
        }

        return m_data.contains(key);
    }

    iterator find(const std::string &key)
    {
        return m_data.find(key);
    }

    [[nodiscard]] const_iterator cbegin() const
    {
        return m_data.cbegin();
    }

    [[nodiscard]] const_iterator cend() const
    {
        return m_data.cend();
    }

    [[nodiscard]] iterator begin()
    {
        return m_data.begin();
    }

    [[nodiscard]] iterator end()
    {
        return m_data.end();
    }

    explicit Config(const std::string &path);

    /**
            * @brief Get the value of the key
            * @param key
            * @return value
            */
    template<typename T>
    T get(const std::string &key) const
    {
        // Check if key is nested
        if (key.find('.') != std::string::npos) {
            std::string group = key.substr(0, key.find('.'));
            std::string name = key.substr(key.find('.') + 1);

            if (m_data.find(group) == m_data.end()) {
                throw std::runtime_error("Config does not contain group name: " + group);
            }

            return m_data[group][name].value<T>().value();
        }

        return m_data[key].value<T>().value();
    }

    [[nodiscard]] std::string getByIdx(size_t idx) const
    {
        auto it = m_data.begin();
        std::advance(it, idx);

        return it->second.value<std::string>().value();
    }

    [[nodiscard]] size_t size() const
    {
        return m_data.size();
    }

private:
    toml::table m_data;
};

}// namespace ros2monitor
