#pragma once

#include "daemon_client/DaemonClient.hpp"
#include <QAbstractListModel>

namespace ros2monitor {
class DaemonClientModel : public QAbstractListModel {
    Q_OBJECT
public:
    explicit DaemonClientModel(QObject *parent = nullptr);

    void setState(std::shared_ptr<Ros2State> state);
    void setDaemonClient(std::shared_ptr<DaemonClient> daemon_client);
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
    int rowCount(const QModelIndex &parent = QModelIndex()) const override;

public slots:
    void killNode(const QString &node_name);

private:
    std::shared_ptr<DaemonClient> m_daemon_client;
    std::shared_ptr<Ros2State> m_state;
};
}
