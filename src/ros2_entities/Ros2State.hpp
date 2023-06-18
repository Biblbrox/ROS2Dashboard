#pragma once


#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "Ros2Entity.hpp"
#include "Ros2Connection.hpp"
#include "Ros2Node.hpp"
#include "Ros2Package.hpp"
#include "Ros2Topic.hpp"
#include "Ros2Publisher.hpp"
#include "Ros2Subscriber.hpp"

namespace ros2monitor {

    struct Ros2Storage {
        std::vector<Ros2Package> packages;
        std::vector<Ros2Node> nodes;
        std::vector<Ros2Topic> topics;
        std::vector<Ros2Subscriber> subscribers;
        std::vector<Ros2Publisher> publishers;
        std::vector<Ros2Connection> connections;
    };

    class Ros2State {
    public:
        explicit Ros2State(std::string_view state);

        void update(std::string_view new_state);

        /**
         * Get node with name node_name
         * @param node_name
         * @return
         */
        Ros2Node node(std::string_view node_name) const;
        Ros2Package package(std::string_view package_name) const;
        Ros2Topic topic(std::string_view package_name) const;

        /**
         * Get all running nodes
         * @return
         */
        std::vector<Ros2Node> nodes() const;
        std::vector<Ros2Package> packages() const;
        std::vector<Ros2Connection> connections() const;

        /**
         * Get all topics of active (running) nodes
         * @return
         */
        std::vector<Ros2Topic> topics() const;

    private:
        Ros2Storage m_coldState;// All packages, nodes, executables and so on
        Ros2Storage m_hotState;// Currently running nodes, executables, ...
    };

}