#include <QIcon>
#include <QQuickVTKItem.h>
#include <QQuickVTKRenderWindow.h>

#include "Application.hpp"
#include "QuickQanava"
#include "core/Logger.hpp"
#include "qml/visualization/GenericTextViz.hpp"

#include "GeometryViz.hpp"
#include "qml/models/DaemonClientModel.hpp"

using std::make_shared;

ros2monitor::Application::Application(int &argc, char **argv)
{
    QQuickVTKItem::setGraphicsApi();
    //QtWebEngineQuick::initialize();
    m_guiApp = make_shared<QGuiApplication>(argc, argv);
    m_guiApp->setWindowIcon(QIcon(":/ui/icons/ROS2Dashboard.svg"));
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

    auto update_entities = [this](const QString &data) {
        auto state = make_shared<Ros2State>(data.toStdString());
        m_nodeListModel->updateState(state);
        m_packageListModel->updateState(state);
        m_topic_model->updateState(state);
        m_daemon_client_model->setState(state);
        m_connectionListModel->update(state->connections());
    };

    QObject::connect(this, &Application::hotStateUpdated, [update_entities](const QString &data) {
        update_entities(data);
    });
    QObject::connect(m_daemon_client_model.get(), &DaemonClientModel::hotStateUpdated, [update_entities](const QString &data) {
        update_entities(data);
    });

    std::future<std::string> state_future = m_daemonClient->stateRequestAsync();
    std::string response = state_future.get();
    emit hotStateUpdated(QString::fromStdString(response));
}

int ros2monitor::Application::run()
{
    try {
        //QQuickStyle::setStyle("Material");
        QuickQanava::initialize(m_qmlEngine.get());
        qmlRegisterSingletonType(QUrl("qrc:///ui/Theme.qml"), "Theme", 1, 0, "Theme");
        qmlRegisterType<viz::GeometryViz>("com.vtk.types", 1, 0, "GeometryViz");
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
