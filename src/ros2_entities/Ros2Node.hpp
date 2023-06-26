#pragma once

#include <string>
#include <vector>

#include "Ros2Publisher.hpp"
#include "Ros2Subscriber.hpp"

namespace ros2monitor {
struct Ros2Node {
    Ros2Node(std::string name_, std::string package_name_) : name(std::move(name_)), package_name(std::move(package_name_))
    {
    }

    std::string name;
    std::string package_name;

    std::vector<Ros2Subscriber> subscribers;
    std::vector<Ros2Publisher> publishers;
};

}