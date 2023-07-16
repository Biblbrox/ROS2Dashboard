#pragma once

#include <QFuture>
#include <QQmlApplicationEngine>
#include <QQuickPaintedItem>
#include <QtCore>
#include <rclcpp/node.hpp>
#include <tuple>
#include <utility>
#include <vector>

#include "VizComponent.hpp"
#include "ros2_entities/Ros2Connection.hpp"
#include "src/network/DaemonClient.hpp"

namespace ros2monitor {

enum class VisualizationType {
    /**
     * Raster type can be represented with such message types as std_msgs::msg::Image
     * Geometry type describe point cloud related types
     * String type is about simple message structures, which can be easily converted to string
     */

    raster,
    geometry,
    string
};

enum class VisualizerState {
    running,
    paused
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
    explicit VisualizerModel(int argc, char **argv, std::shared_ptr<DaemonClient> daemon_client, QObject *parent = nullptr);
    ~VisualizerModel() override;

    void update(std::vector<Ros2Connection> connections);

    void addTopicViz(VisualizationType type, const std::string &topic_type, const std::string &topic_name, QQuickItem *item);
    bool inTextGroup(const std::string &topic_type);
    bool inRasterGroup(const std::string &topic_type);
    bool inGeometryGroup(const std::string &topic_type);

    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    QHash<int, QByteArray> roleNames() const override;

public slots:
    bool hasTopicViz(const QString &topic_name);
    QString getTopicCategory(const QString &topic_type);
    void pauseViz(const QString &topic_name);
    void resumeViz(const QString &topic_name);
    void removeViz(const QString &topic_name);

signals:
    void topicVizRemoved(const QString &topic_name);

private:
    std::vector<Ros2Connection> m_connections;
    std::unordered_map<std::string, QQuickItem *> m_components;
    std::shared_ptr<rclcpp::Node> m_visualizerNode;
    std::unordered_map<std::string, std::shared_ptr<rclcpp::GenericSubscription>> m_subscribers;
    std::unordered_map<std::string, VisualizerState> m_visualizer_states;
    QFuture<void> m_nodeFuture;
    std::shared_ptr<DaemonClient> m_daemon_client;
    std::vector<std::string> m_textGroup;
    std::vector<std::string> m_rasterGroup;
    std::vector<std::string> m_geometryGroup;
    int m_argc;
    char **m_argv;
};


}

Q_DECLARE_METATYPE(ros2monitor::TopicQml)