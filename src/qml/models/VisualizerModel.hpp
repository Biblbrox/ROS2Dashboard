#pragma once

#include "ros2_entities/Ros2Connection.hpp"
#include <QFuture>
#include <QtCore>
#include <rclcpp/node.hpp>

#include "VizComponent.hpp"
#include "daemon_client/DaemonClient.hpp"
#include <QQmlApplicationEngine>
#include <QQuickPaintedItem>
#include <tuple>
#include <utility>
#include <vector>

namespace ros2monitor {

enum class VisualizationType {
    image,
    point_cloud,
    text
};

enum VisualizerRole {
    ComponentRoleName = Qt::UserRole + 1,
    TopicRoleName
};

class TopicQml : public QObject {
    Q_OBJECT
    Q_PROPERTY(QString name READ name WRITE setName)
    Q_PROPERTY(QString nodeName READ nodeName WRITE setNodeName)
    Q_PROPERTY(QString type READ type WRITE setType)
    QML_VALUE_TYPE(Topic)

public:
    TopicQml(QObject *parent = nullptr) : QObject(parent)
    {
    }

    void setName(QString name)
    {
        m_name = std::move(name);
    }

    void setNodeName(QString name)
    {
        m_nodeName = std::move(name);
    }

    void setType(QString type)
    {
        m_type = std::move(type);
    }

    QString name()
    {
        return m_name;
    }

    QString nodeName()
    {
        return m_nodeName;
    }

    QString type()
    {
        return m_type;
    }

private:
    QString m_name;
    QString m_nodeName;
    QString m_type;
};

class VisualizerModel : public QAbstractListModel {
    Q_OBJECT
public:
    explicit VisualizerModel(int argc, char** argv, std::shared_ptr<DaemonClient> daemon_client, QObject *parent = nullptr);
    ~VisualizerModel() override;

    void update(std::vector<Ros2Connection> connections);

    void addTopicViz(VisualizationType type, const std::string &topic_name, VizComponent *item);
    void removeViz(const std::string &topic_name);

    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    QHash<int, QByteArray> roleNames() const override;

private:
    std::vector<Ros2Connection> m_connections;
    std::unordered_map<std::string, VizComponent *> m_components;
    std::shared_ptr<rclcpp::Node> m_visualizerNode;
    std::unordered_map<std::string, std::shared_ptr<rclcpp::SubscriptionBase>> m_subscribers;
    QFuture<void> m_nodeFuture;
    std::shared_ptr<DaemonClient> m_daemon_client;
    int m_argc;
    char** m_argv;
};


}

Q_DECLARE_METATYPE(ros2monitor::TopicQml)