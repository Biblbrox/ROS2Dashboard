#pragma once

#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <memory>

#include "Ros2TopicListModel.hpp"
#include "daemon_client/DaemonClient.hpp"
#include "qml/models/Ros2ConnectionListModel.hpp"
#include "qml/models/Ros2NodeListModel.hpp"
#include "qml/models/Ros2PackageListModel.hpp"
#include "src/qml/models/VisualizerModel.hpp"

namespace ros2monitor {
class Application  {
public:
    explicit Application(int& argc, char **argv);

    void registerModels();

    int run();

private:
    std::shared_ptr<DaemonClient> m_daemonClient;
    std::shared_ptr<QGuiApplication> m_guiApp;
    std::shared_ptr<QQmlApplicationEngine> m_qmlEngine;

    std::shared_ptr<Ros2NodeListModel> m_nodeListModel;
    std::shared_ptr<Ros2PackageListModel> m_packageListModel;
    std::shared_ptr<Ros2ConnectionListModel> m_connectionListModel;
    std::shared_ptr<Ros2TopicListModel> m_topic_model;
    std::shared_ptr<VisualizerModel> m_visualizer_model;
};
}