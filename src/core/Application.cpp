#include <QtWebEngineQuick/QtWebEngineQuick>

#include "Application.hpp"
#include "QuickQanava"

using std::make_shared;

ros2monitor::Application::Application(int argc, char **argv) {
    QtWebEngineQuick::initialize();
    m_guiApp = make_shared<QGuiApplication>(argc, argv);
    m_qmlEngine = make_shared<QQmlApplicationEngine>();
    m_daemonClient = make_shared<DaemonClient>("/tmp/ros2monitor.sock");

    m_nodeListModel = make_shared<Ros2NodeListModel>(m_guiApp.get());
    m_packageListModel = make_shared<Ros2PackageListModel>(m_guiApp.get());
    m_connectionListModel = make_shared<Ros2ConnectionListModel>(m_guiApp.get());

    QObject::connect(m_daemonClient.get(), &DaemonClient::hotStateUpdated, [this](const QString &data) {
        auto state = make_shared<Ros2State>(data.toStdString());
        m_nodeListModel->updateState(state);
        m_packageListModel->updateState(state);
        m_connectionListModel->update(state->connections());
    });

    registerModels();
    m_daemonClient->receiveState();
}

int ros2monitor::Application::run() {
    //QQuickStyle::setStyle("Material");
    QuickQanava::initialize(m_qmlEngine.get());
    qmlRegisterSingletonType(QUrl("qrc:///ui/Theme.qml"), "Theme", 1, 0, "Theme");
    m_qmlEngine->load("qrc:///ui/Main.qml");
    int code = m_guiApp->exec();
    return code;
}

void ros2monitor::Application::registerModels() {
    m_qmlEngine->rootContext()->setContextProperty("nodeListModel", m_nodeListModel.get());
    m_qmlEngine->rootContext()->setContextProperty("packageListModel", m_packageListModel.get());
    m_qmlEngine->rootContext()->setContextProperty("connectionListModel", m_connectionListModel.get());
}
