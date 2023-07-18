#pragma once

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "Ros2Publisher.hpp"
#include "Ros2Subscriber.hpp"
#include "network/Host.hpp"

namespace ros2monitor {

enum class Ros2NodeStatus {
    unconfigured,
    inactive,
    active,
    shutdown,
    // For non lifecycle nodes, which doesn't support lifecycle states
    non_lifecycle
};

namespace detail {
const std::unordered_map<std::string, Ros2NodeStatus> ros2_node_status_map = {
    { "unconfigured", Ros2NodeStatus::unconfigured },
    { "inactive", Ros2NodeStatus::inactive },
    { "active", Ros2NodeStatus::active },
    { "shutdown", Ros2NodeStatus::shutdown },
    { "nonlifecycle", Ros2NodeStatus::non_lifecycle }
};
inline std::string str_tolower(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(),
                   // static_cast<int(*)(int)>(std::tolower)         // wrong
                   // [](int c){ return std::tolower(c); }           // wrong
                   // [](char c){ return std::tolower(c); }          // wrong
                   [](unsigned char c) { return std::tolower(c); }// correct
    );
    return s;
}
}

inline Ros2NodeStatus str2NodeStatus(std::string status)
{
    std::string status_lower = detail::str_tolower(std::move(status));
    return detail::ros2_node_status_map.at(status_lower);
}

inline std::string nodeStatus2Str(Ros2NodeStatus status)
{
    for (const auto &[key, value] : detail::ros2_node_status_map) {
        if (value == status) {
            return key;
        }
    }
    return "unknown";

}
struct Ros2Node {
    Ros2Node(std::string name_, std::string package_name_, Host host_, Ros2NodeStatus status_) : name(std::move(name_)), package_name(std::move(package_name_)), host(std::move(host_)), status(status_)
    {
    }

    std::string name;
    std::string package_name;
    Host host;

    std::vector<Ros2Subscriber> subscribers;
    std::vector<Ros2Publisher> publishers;

    Ros2NodeStatus status;
};

}