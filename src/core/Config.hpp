#pragma once

#define TOML_EXCEPTIONS 0// Disable exceptions
#include <toml++/toml.h>

namespace ros2monitor {

template<typename T, typename Path>
auto build_from_path(T &&value, Path &&path_component)
{
    using component_type = std::remove_cv_t<std::remove_reference_t<Path>>;
    static_assert(std::is_integral_v<component_type> || toml::is_key_or_convertible<Path &&>,
                  "path components must be integers or strings");

    // making an array
    if constexpr (std::is_integral_v<component_type>) {
        toml::array arr;
        const auto index = static_cast<std::size_t>(path_component);
        arr.reserve(index + 1u);

        // backfill with integers
        while (arr.size() < index)
            arr.push_back(0);

        // add the actual value
        arr.push_back(static_cast<T &&>(value));

        return arr;
    }

    // making a table
    else {
        toml::table tbl;

        tbl.insert_or_assign(static_cast<Path &&>(path_component), static_cast<T &&>(value));

        return tbl;
    }
}

template<typename T, typename Path, typename... Paths>
auto build_from_path(T &&value, Path &&path_component, Paths &&...path_components)
{
    static_assert(sizeof...(Paths));

    return build_from_path(build_from_path(static_cast<T &&>(value), static_cast<Paths &&>(path_components)...),
                           static_cast<Path &&>(path_component));
}

static void merge_left(toml::table &lhs, toml::table &&rhs);

static void merge_left(toml::array &lhs, toml::array &&rhs)
{
    rhs.for_each(
            [&](std::size_t index, auto &&rhs_val) {
                // rhs index not found in lhs - direct move
                if (lhs.size() <= index) {
                    lhs.push_back(std::move(rhs_val));
                    return;
                }

                // both elements were the same container type -  recurse into them
                if constexpr (toml::is_container<decltype(rhs_val)>) {
                    using rhs_type = std::remove_cv_t<std::remove_reference_t<decltype(rhs_val)>>;
                    if (auto lhs_child = lhs[index].as<rhs_type>()) {
                        merge_left(*lhs_child, std::move(rhs_val));
                        return;
                    }
                }

                // replace lhs element with rhs
                lhs.replace(lhs.cbegin() + index, std::move(rhs_val));
            });
}

static void merge_left(toml::table &lhs, toml::table &&rhs)
{
    rhs.for_each(
            [&](const toml::key &rhs_key, auto &&rhs_val) {
                auto lhs_it = lhs.lower_bound(rhs_key);

                // rhs key not found in lhs - direct move
                if (lhs_it == lhs.cend() || lhs_it->first != rhs_key) {
                    using rhs_type = std::remove_cv_t<std::remove_reference_t<decltype(rhs_val)>>;
                    lhs.emplace_hint<rhs_type>(lhs_it, rhs_key, std::move(rhs_val));
                    return;
                }

                // both children were the same container type -  recurse into them
                if constexpr (toml::is_container<decltype(rhs_val)>) {
                    using rhs_type = std::remove_cv_t<std::remove_reference_t<decltype(rhs_val)>>;
                    if (auto lhs_child = lhs_it->second.as<rhs_type>()) {
                        merge_left(*lhs_child, std::move(rhs_val));
                        return;
                    }
                }

                // replace lhs value with rhs
                lhs.insert_or_assign(rhs_key, std::move(rhs_val));
            });
}

template<typename T, typename Path, typename... Paths>
void insert_at_path(toml::table &root, T &&value, Path &&path_component, Paths &&...path_components)
{
    auto rhs = build_from_path(static_cast<T &&>(value),
                               static_cast<Path &&>(path_component),
                               static_cast<Paths &&>(path_components)...);

    if constexpr (toml::is_array<decltype(rhs)>) {
        merge_left(root, toml::table{ "", std::move(rhs) });
    } else {
        merge_left(root, std::move(rhs));
    }
}

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

    template<typename ValueType>
    void set(const std::string &key_path, ValueType value)
    {
        // Check if key is nested
        if (key_path.find('.') != std::string::npos) {
            std::string group = key_path.substr(0, key_path.find('.'));
            std::string name = key_path.substr(key_path.find('.') + 1);

            if (m_data.find(group) == m_data.end()) {
                throw std::runtime_error("Config does not contain group name: " + group);
            }

            insert_at_path(m_data, value, group, name);// inserts at a.b.c.d[2]

            return;
        }

        m_data.insert_or_assign(key_path, value);
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

    void save(const std::string &path);

    [[nodiscard]] std::string path() const;

private:
    toml::table m_data;
    std::string m_path;
};

}// namespace ros2monitor
