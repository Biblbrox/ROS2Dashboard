#pragma once

#include <string>

namespace ros2monitor {
    struct Ros2Topic {
        Ros2Topic(std::string name_, std::string node_name_) : name(std::move(name_)),
                                                               node_name(std::move(node_name_)) {
        }

        std::string name;
        std::string node_name;
    };

}