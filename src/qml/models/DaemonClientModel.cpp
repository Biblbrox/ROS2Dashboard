#include "DaemonClientModel.hpp"

#include <utility>

namespace ros2monitor {

DaemonClientModel::DaemonClientModel(QObject *parent) : QObject(parent)
{
}

void DaemonClientModel::killNode(const QString &node_name)
{
    m_daemon_client->killNodeRequest(node_name.toStdString());
}

void DaemonClientModel::setState(std::shared_ptr<Ros2State> state)
{
    m_state = std::move(state);
}

void DaemonClientModel::setDaemonClient(std::shared_ptr<DaemonClient> daemon_client)
{
    m_daemon_client = std::move(daemon_client);
}

/*QVariant DaemonClientModel::data(const QModelIndex &index, int role) const
{
    return QVariant();
}

int DaemonClientModel::rowCount(const QModelIndex &parent) const
{
    return 0;
}*/

void DaemonClientModel::update()
{
    std::future<std::string> state_future = m_daemon_client->stateRequestAsync();
    std::string response = state_future.get();
    emit hotStateUpdated(QString::fromStdString(response));
}

}