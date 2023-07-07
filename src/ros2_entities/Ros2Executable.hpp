#pragma once

#include "Ros2Entity.hpp"
#include <string>
namespace ros2monitor {

struct Ros2Executable {

    Ros2Executable(std::string name_, std::string package_name_, std::string path_) : path(std::move(path_)), name(std::move(name_)), package_name(std::move(package_name_))
    {
    }


    std::string path;
    std::string name;
    std::string package_name;
};
}
