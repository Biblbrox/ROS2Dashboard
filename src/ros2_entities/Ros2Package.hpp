#pragma once

#include "Ros2Entity.hpp"
#include <string>

namespace ros2monitor {

struct Ros2Package {
    Ros2Package(std::string name_, std::string path_) : name(std::move(name_)), path(std::move(path_))
    {
    }

    std::string name;
    std::string path;
};
}
