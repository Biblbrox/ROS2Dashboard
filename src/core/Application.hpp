#pragma once

#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <memory>

#include "daemon_client/DaemonClient.hpp"
#include "qml/models/Ros2NodeListModel.hpp"
#include "qml/models/Ros2PackageListModel.hpp"
#include "qml/models/Ros2ConnectionListModel.hpp"

namespace ros2monitor {
class Application  {
public:
    explicit Application(int argc, char **argv);

    void registerModels();

    int run();

private:
    std::shared_ptr<DaemonClient> m_daemonClient;
    std::shared_ptr<QGuiApplication> m_guiApp;
    std::shared_ptr<QQmlApplicationEngine> m_qmlEngine;

    std::shared_ptr<Ros2NodeListModel> m_nodeListModel;
    std::shared_ptr<Ros2PackageListModel> m_packageListModel;
    std::shared_ptr<Ros2ConnectionListModel> m_connectionListModel;
};
}