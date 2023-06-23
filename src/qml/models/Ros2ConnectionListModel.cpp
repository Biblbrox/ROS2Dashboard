#include "Ros2ConnectionListModel.hpp"

namespace ros2monitor {

Ros2ConnectionListModel::Ros2ConnectionListModel(QObject *parent)
    : QAbstractListModel(parent)
{
    m_string2Role["topic_name"] = TopicNameRole;
    m_string2Role["src_node_name"] = SrcNodeNameRole;
    m_string2Role["dst_node_name"] = DstNodeNameRole;
}

QVariant Ros2ConnectionListModel::data(const QModelIndex &index, int role) const
{
    if (m_connections.empty() || !index.isValid())
        return QVariant{};
    QVariant value;
    if (role == TopicNameRole) {
        value = QString(m_connections[index.row()].topic_name.c_str());
    } else if (role == SrcNodeNameRole) {
        value = QString(m_connections[index.row()].src_node_name.c_str());
    } else if (role == DstNodeNameRole) {
        value = QString(m_connections[index.row()].dst_node_name.c_str());
    }
    return value;
}

int Ros2ConnectionListModel::rowCount(const QModelIndex &parent) const
{
    return static_cast<int>(m_connections.size());
}

QHash<int, QByteArray> Ros2ConnectionListModel::roleNames() const
{
    QHash<int, QByteArray> roles;
    roles[TopicNameRole] = "topic_name";
    roles[SrcNodeNameRole] = "src_node_name";
    roles[DstNodeNameRole] = "dst_node_name";
    return roles;
}

void Ros2ConnectionListModel::update(std::vector<Ros2Connection> connections)
{
    m_connections = std::move(connections);
}

QVariant Ros2ConnectionListModel::getRow(int i, const QString &role_name)
{
    assert(m_string2Role.contains(role_name.toStdString()));
    return data(index(i, 0), m_string2Role[role_name.toStdString()]);
}
}
