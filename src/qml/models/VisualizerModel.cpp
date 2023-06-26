#include <QtConcurrent/QtConcurrent>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <std_msgs/msg/string.hpp>
#include <utility>


#include "VideoViz.hpp"
#include "VisualizerModel.hpp"
#include "core/Logger.hpp"

using rclcpp::Node;
using sensor_msgs::msg::Image;
using sensor_msgs::msg::PointCloud;
using std::make_shared;
using std::make_unique;
using std::unique_ptr;
using std_msgs::msg::String;

namespace ros2monitor {

VisualizerModel::VisualizerModel(int argc, char **argv, std::shared_ptr<DaemonClient> daemon_client, QObject *parent) : QAbstractListModel(parent), m_argc(argc), m_argv(argv)
{
    m_daemon_client = std::move(daemon_client);
}

void VisualizerModel::update(std::vector<Ros2Connection> connections)
{
    m_connections = std::move(connections);
}

void VisualizerModel::addTopicViz(VisualizationType type, const std::string &topic_name, VizComponent *item)
{
    if (m_components.contains(topic_name))
        return;

    if (m_visualizerNode == nullptr) {
        rclcpp::init(m_argc, m_argv);
        const std::string reserved_name = "visualization_node";
        m_visualizerNode = make_shared<Node>(reserved_name);
    }

    if (type == VisualizationType::image) {
        m_components[topic_name] = item;
        m_subscribers[topic_name] = m_visualizerNode->create_subscription<Image>(topic_name, 10, [this, topic_name](Image::ConstSharedPtr image) {
            m_components[topic_name]->updateData("asd");
        });
    } else if (type == VisualizationType::point_cloud) {
        m_components[topic_name] = item;
        m_subscribers[topic_name] = m_visualizerNode->create_subscription<PointCloud>(topic_name, 10, [this, topic_name](PointCloud::ConstSharedPtr cloud) {
            m_components[topic_name]->updateData("asd");
        });
    } else if (type == VisualizationType::text) {
        m_components[topic_name] = item;

        m_subscribers[topic_name] = m_visualizerNode->create_subscription<String>(topic_name, 10, [this, topic_name](const String::SharedPtr str) {
            m_components[topic_name]->updateData(str->data.c_str());
        });
    }


    if (!m_nodeFuture.isRunning()) {
        m_daemon_client->killNode("visualization_node");
        m_nodeFuture = QtConcurrent::run([this]() {
            Logger::debug("Running node");
            rclcpp::spin(m_visualizerNode);
            rclcpp::shutdown();
        });
    }
}

void VisualizerModel::removeViz(const std::string &topic_name)
{
    if (!m_components.contains(topic_name))
        return;

    auto it = m_components.find(topic_name);
    m_components.erase(it);
}

QVariant VisualizerModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid())
        return QVariant{};

    size_t idx = index.row();
    auto it = m_components.begin();
    std::advance(it, idx);

    QVariant value;

    if (role == TopicRoleName) {
        /*TopicQml topic(parent());
        topic.setName()
        value.setValue()*/
    }
    return value;
}

int VisualizerModel::rowCount(const QModelIndex &parent) const
{
    return m_components.size();
}

QHash<int, QByteArray> VisualizerModel::roleNames() const
{
    QHash<int, QByteArray> roles;
    //roles[ComponentRoleName] = "component";
    roles[TopicRoleName] = "topic";
    return roles;
}

VisualizerModel::~VisualizerModel()
{
    m_nodeFuture.waitForFinished();
    rclcpp::shutdown();
}
}