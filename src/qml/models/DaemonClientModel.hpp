#pragma once

#include <QAbstractListModel>

#include "network/DaemonClient.hpp"

namespace ros2monitor {
class DaemonClientModel : public QObject {
    Q_OBJECT
public:
    explicit DaemonClientModel(QObject *parent = nullptr);

    void setState(std::shared_ptr<Ros2State> state);
    void setDaemonClient(std::shared_ptr<DaemonClient> daemon_client);
    //QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
    //int rowCount(const QModelIndex &parent = QModelIndex()) const override;

signals:
    void hotStateUpdated(const QString &jsonState);

public slots:
    void killNode(const QString &node_name);
    void update();

private:
    std::shared_ptr<DaemonClient> m_daemon_client;
    std::shared_ptr<Ros2State> m_state;
};
}
