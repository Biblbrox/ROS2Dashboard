#pragma once


#include <string>

namespace ros2monitor {
    struct Ros2Subscriber {
        Ros2Subscriber(std::string topic_name_, std::string node_name_) : topic_name(std::move(topic_name_)),
                                                                          node_name(std::move(node_name_)) {
        }

        std::string topic_name;
        std::string node_name;
    };

}