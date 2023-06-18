#pragma once

#include <string>

namespace ros2monitor {

    struct Ros2Connection {
        Ros2Connection(std::string topic_name_, std::string src_node_name_, std::string dst_node_name_) : topic_name(
                std::move(topic_name_)), src_node_name(std::move(src_node_name_)), dst_node_name(
                std::move(dst_node_name_)) {}

        std::string topic_name;
        std::string src_node_name;
        std::string dst_node_name;
    };

}
