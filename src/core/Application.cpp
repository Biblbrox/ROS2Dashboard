#include <QtWebEngineQuick/QtWebEngineQuick>

#include "Application.hpp"
#include "QuickQanava"
#include "core/Logger.hpp"
#include "qml/visualization/GenericTextViz.hpp"

#include "qml/models/DaemonClientModel.hpp"

using std::make_shared;

ros2monitor::Application::Application(int &argc, char **argv)
{
    QtWebEngineQuick::initialize();
    m_guiApp = make_shared<QGuiApplication>(argc, argv);
    m_qmlEngine = make_shared<QQmlApplicationEngine>();
    m_daemonClient = make_shared<DaemonClient>("/tmp/ros2monitor.sock");

    m_nodeListModel = make_shared<Ros2NodeListModel>(m_guiApp.get());
    m_packageListModel = make_shared<Ros2PackageListModel>(m_guiApp.get());
    m_connectionListModel = make_shared<Ros2ConnectionListModel>(m_guiApp.get());
    m_topic_model = make_shared<Ros2TopicListModel>(m_guiApp.get());
    m_visualizer_model = make_shared<VisualizerModel>(argc, argv, m_daemonClient, m_guiApp.get());
    m_daemon_client_model = make_shared<DaemonClientModel>(m_guiApp.get());
    m_daemon_client_model->setDaemonClient(m_daemonClient);

    registerModels();

    QObject::connect(m_daemonClient.get(), &DaemonClient::hotStateUpdated, [this](const QString &data) {
        auto state = make_shared<Ros2State>(data.toStdString());
        m_nodeListModel->updateState(state);
        m_packageListModel->updateState(state);
        m_topic_model->updateState(state);
        m_daemon_client_model->setState(state);
        m_connectionListModel->update(state->connections());
    });

    m_daemonClient->receiveState();
}

int ros2monitor::Application::run()
{
    try {
        //QQuickStyle::setStyle("Material");
        QuickQanava::initialize(m_qmlEngine.get());
        qmlRegisterSingletonType(QUrl("qrc:///ui/Theme.qml"), "Theme", 1, 0, "Theme");
        m_qmlEngine->load("qrc:///ui/Main.qml");
    } catch (std::exception &e) {
        Logger::error(fmt::format("Unable to initialize application. Error: {}", e.what()));
        return -1;
    }
    int code = m_guiApp->exec();
    return code;
}

void ros2monitor::Application::registerModels()
{
    m_qmlEngine->rootContext()->setContextProperty("nodeListModel", m_nodeListModel.get());
    m_qmlEngine->rootContext()->setContextProperty("packageListModel", m_packageListModel.get());
    m_qmlEngine->rootContext()->setContextProperty("connectionListModel", m_connectionListModel.get());
    m_qmlEngine->rootContext()->setContextProperty("visualizerModel", m_visualizer_model.get());
    m_qmlEngine->rootContext()->setContextProperty("topicListModel", m_topic_model.get());
    m_qmlEngine->rootContext()->setContextProperty("daemonClientModel", m_daemon_client_model.get());
}
