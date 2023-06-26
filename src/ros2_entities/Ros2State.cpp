#include <fmt/format.h>
#include <nlohmann/json.hpp>
#include <utility>

#include "Ros2State.hpp"
#include "utils/StrUtils.hpp"

namespace ros2monitor {

using json = nlohmann::json;
using std::find_if;

void Ros2State::update(std::string jsonState)
{
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
    trim(jsonState);
    json state = json::parse(jsonState);
    if (state.contains("packages")) {
        for (const auto &packageJson: state["packages"]) {
            if (!packageJson.contains("name") || !packageJson.contains("path")) {
                throw std::invalid_argument(fmt::format("Invalid json response. Package object must include both name and path properties"));
            }

            m_hotState.packages.emplace_back(packageJson["name"], packageJson["path"]);
        }
    }

    if (state.contains("nodes")) {
        for (const auto &nodeJson: state["nodes"]) {
            if (nodeJson["name"] == "/visualization_node")
                continue;

            if (!nodeJson.contains("name") || !nodeJson.contains("package_name")) {
                throw std::invalid_argument(fmt::format("Invalid json response. Node object must include both name and package_name properties. Current json: {}", nodeJson.dump()));
            }

            Ros2Node node(nodeJson["name"], nodeJson["package_name"]);
            std::vector<Ros2Subscriber> subscribers;
            std::vector<Ros2Publisher> publishers;
            // Extract subscribers
            for (const auto &subscriber_info: nodeJson["subscribers"])
                subscribers.emplace_back(subscriber_info["topic_name"], node.name);

            for (const auto &publisher_info: nodeJson["publishers"])
                publishers.emplace_back(publisher_info["topic_name"], node.name);
            node.publishers = publishers;
            node.subscribers = subscribers;
            m_hotState.nodes.push_back(node);
        }
    }

    /// Update topics
    if (state.contains("topics")) {
        for (const auto &topicJson: state["topics"]) {
            if (!topicJson.contains("name") || !topicJson.contains("node_name") || !topicJson.contains("topic_type")) {
                throw std::invalid_argument(fmt::format("Invalid json response. Topic object must include name, node_name and topic type properties. Current json: {}", topicJson.dump()));
            }

            m_hotState.topics.emplace_back(topicJson["name"], topicJson["node_name"], topicJson["topic_type"]);
        }
    }

    /// Next, we need to update our connections
    m_hotState.connections.clear();
    for (const auto &node1: m_hotState.nodes) {
        for (const auto &node2: m_hotState.nodes) {
            if (node1.name == node2.name)
                continue;

            for (const auto &subscriber: node1.subscribers) {
                for (const auto &publisher: node2.publishers) {
                    if (subscriber.topic_name == publisher.topic_name)
                        m_hotState.connections.emplace_back(subscriber.topic_name, publisher.node_name,
                                                            subscriber.node_name);
                }
            }
        }
    }
}

std::vector<Ros2Node> Ros2State::nodes() const
{
    return m_hotState.nodes;
}

std::vector<Ros2Package> Ros2State::packages() const
{
    return m_hotState.packages;
}

Ros2State::Ros2State(std::string state)
{
    update(std::move(state));
}

std::vector<Ros2Connection> Ros2State::connections() const
{
    return m_hotState.connections;
}

std::vector<Ros2Topic> Ros2State::topics() const
{
    return m_hotState.topics;
}


Ros2Node Ros2State::node(std::string_view node_name) const
{
    return *find_if(m_hotState.nodes.cbegin(), m_hotState.nodes.cend(), [node_name](const auto &node) {
        return node.name == node_name;
    });
}

Ros2Package Ros2State::package(std::string_view package_name) const
{
    return *find_if(m_hotState.packages.cbegin(), m_hotState.packages.cend(), [package_name](const auto &package) {
        return package.name == package_name;
    });
}


}