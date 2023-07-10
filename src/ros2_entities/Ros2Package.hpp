#pragma once

#include <string>
#include <vector>

#include "Ros2Executable.hpp"

namespace ros2monitor {

struct Ros2Package {
    Ros2Package(std::string name_, std::string path_, std::vector<Ros2Executable> executables_) : name(std::move(name_)), path(std::move(path_)), executables(std::move(executables_))
    {
    }

    std::string name;
    std::string path;
    std::vector<Ros2Executable> executables;
};
}
