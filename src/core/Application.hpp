#pragma once

#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <memory>

#include "DaemonClientModel.hpp"
#include "Ros2TopicListModel.hpp"
#include "SettingsModel.hpp"
#include "qml/models/Ros2ConnectionListModel.hpp"
#include "qml/models/Ros2NodeListModel.hpp"
#include "qml/models/Ros2PackageListModel.hpp"
#include "qml/visualization/RasterViz.hpp"
#include "src/network/DaemonClient.hpp"
#include "src/qml/models/VisualizerModel.hpp"

namespace ros2monitor {
class Application : public QObject {
    Q_OBJECT
public:
    explicit Application(int &argc, char **argv);

    void registerModels();

    int run();
signals:
    void hotStateUpdated(const QString &jsonState);

private:
    std::shared_ptr<DaemonClient> m_daemon_client;
    std::shared_ptr<QGuiApplication> m_guiApp;
    std::shared_ptr<QQmlApplicationEngine> m_qmlEngine;
    std::shared_ptr<Config> m_config;

    std::shared_ptr<Ros2NodeListModel> m_nodeListModel;
    std::shared_ptr<Ros2PackageListModel> m_packageListModel;
    std::shared_ptr<Ros2ConnectionListModel> m_connectionListModel;
    std::shared_ptr<Ros2TopicListModel> m_topic_model;
    std::shared_ptr<VisualizerModel> m_visualizer_model;
    std::shared_ptr<DaemonClientModel> m_daemon_client_model;
    std::shared_ptr<SettingsModel> m_settings_model;
};
}