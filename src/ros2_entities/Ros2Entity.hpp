#pragma once

#include <string>

namespace ros2monitor {
struct Ros2Entity {
    Ros2Entity(std::string name_, std::string parent_name_) : name(std::move(name_)), parent_name(std::move(parent_name_))
    {
    }

    std::string name;
    std::string parent_name;
};

}