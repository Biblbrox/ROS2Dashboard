#include <nlohmann/json.hpp>

#include "Ros2State.hpp"

namespace ros2monitor {

    using json = nlohmann::json;
    using std::find_if;

    void Ros2State::update(std::string_view jsonState) {
        // We received data, which has structure like this:
        /*
         * {
         *     "packages": [{name: "somename", path: "somepath"}, {name: "somename", path: "somepath"}, ...],
         *     "nodes": [{name: "somename", package_name: "somename"}, {name: "somename", package_name: "somename"}, ...],
         *     ...
         * }
         * Here we are initializing state
         */
        m_hotState.nodes.clear();
        m_hotState.packages.clear();
        m_hotState.topics.clear();
        m_hotState.subscribers.clear();
        m_hotState.publishers.clear();
        json state = json::parse(jsonState);
        if (state.contains("packages")) {
            for (const auto &packageJson: state["packages"]) {
                m_hotState.packages.emplace_back(packageJson["name"], packageJson["path"]);
            }
        }

        if (state.contains("nodes")) {
            for (const auto &nodeJson: state["nodes"]) {
                m_hotState.nodes.emplace_back(nodeJson["name"], nodeJson["package_name"]);
            }
        }

        if (state.contains("topics")) {
            for (const auto &nodeJson: state["topics"]) {
                m_hotState.topics.emplace_back(nodeJson["name"], nodeJson["node_name"]);
            }
        }

        if (state.contains("subscribers")) {
            for (const auto &subscriberJson: state["subscribers"]) {
                m_hotState.subscribers.emplace_back(subscriberJson["topic_name"], subscriberJson["node_name"]);
            }
        }

        if (state.contains("publishers")) {
            for (const auto &publisherJson: state["publishers"]) {
                m_hotState.publishers.emplace_back(publisherJson["topic_name"], publisherJson["node_name"]);
            }
        }

        /// Next, we need to update our connections
        m_hotState.connections.clear();
        for (const auto &subscriber: m_hotState.subscribers) {
            for (const auto &publisher: m_hotState.publishers) {
                if (subscriber.node_name == publisher.node_name)
                    continue;

                if (subscriber.topic_name == publisher.topic_name)
                    m_hotState.connections.emplace_back(subscriber.topic_name, publisher.node_name,
                                                        subscriber.node_name);
            }
        }
    }

    std::vector<Ros2Node> Ros2State::nodes() const {
        return m_hotState.nodes;
    }

    std::vector<Ros2Package> Ros2State::packages() const {
        return m_hotState.packages;
    }

    Ros2State::Ros2State(std::string_view state) {
        update(state);
    }

    std::vector<Ros2Topic> Ros2State::topics() const {
        return m_hotState.topics;
    }

    std::vector<Ros2Connection> Ros2State::connections() const {
        return m_hotState.connections;
    }

    Ros2Node Ros2State::node(std::string_view node_name) const {
        return *find_if(m_hotState.nodes.cbegin(), m_hotState.nodes.cend(), [node_name](const auto &node) {
            return node.name == node_name;
        });
    }

    Ros2Package Ros2State::package(std::string_view package_name) const {
        return *find_if(m_hotState.packages.cbegin(), m_hotState.packages.cend(), [package_name](const auto &package) {
            return package.name == package_name;
        });
    }

    Ros2Topic Ros2State::topic(std::string_view topic_name) const {
        return *find_if(m_hotState.topics.cbegin(), m_hotState.topics.cend(), [topic_name](const auto &topic) {
            return topic.name == topic_name;
        });
    }
}