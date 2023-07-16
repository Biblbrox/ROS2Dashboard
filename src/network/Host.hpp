#pragma once

#include <string>
namespace ros2monitor {

struct Host {
    Host(std::string name_, std::string ip_) : name(std::move(name_)), ip(std::move(ip_))
    {
    }

    std::string name;
    std::string ip;
};

}// namespace ros2monitor
