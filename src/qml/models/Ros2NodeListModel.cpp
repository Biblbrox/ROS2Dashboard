#include "Ros2NodeListModel.hpp"

#include "core/Logger.hpp"
#include "core/Utils.hpp"
#include <utility>


namespace ros2monitor {


Ros2NodeListModel::Ros2NodeListModel(QObject *parent) : QAbstractListModel(parent)
{
    m_role2String[Ros2NodeRole::NameRole] = "name";
    m_role2String[Ros2NodeRole::PackageNameRole] = "package_name";
    m_role2String[Ros2NodeRole::SubscribersNameRole] = "subscribers";
    m_role2String[Ros2NodeRole::PublishersNameRole] = "publishers";
    m_role2String[Ros2NodeRole::DetailInfoRole] = "detail_info";
    m_role2String[Ros2NodeRole::StateNameRole] = "state";
    m_role2String[Ros2NodeRole::HostNameRole] = "host_ip";

    m_string2Role["name"] = Ros2NodeRole::NameRole;
    m_string2Role["package_name"] = Ros2NodeRole::PackageNameRole;
    m_string2Role["subscribers"] = Ros2NodeRole::SubscribersNameRole;
    m_string2Role["publishers"] = Ros2NodeRole::PublishersNameRole;
    m_string2Role["detail_info"] = Ros2NodeRole::DetailInfoRole;
    m_string2Role["state"] = Ros2NodeRole::StateNameRole;
    m_string2Role["host_ip"] = Ros2NodeRole::HostNameRole;
}

QVariant Ros2NodeListModel::data(const QModelIndex &index, int role) const
{
    if (!m_state || !index.isValid())
        return QVariant{};
    Ros2Node node = m_state->nodes()[index.row()];
    QVariant value = getRoleData(node, static_cast<Ros2NodeRole>(role));
    return value;
}

int Ros2NodeListModel::rowCount(const QModelIndex &parent) const
{
    return m_state ? m_state->nodes().size() : 0;
}

QVariant Ros2NodeListModel::getRowByName(int i, const QString &role_name, const QString &entry_name)
{
    assert(m_string2Role.contains(role_name.toStdString()));
    Ros2NodeRole role = m_string2Role[role_name.toStdString()];
    Ros2Node node = m_state->node(entry_name.toLocal8Bit().constData());
    QVariant value = getRoleData(node, role);

    return value;
}

QHash<int, QByteArray> Ros2NodeListModel::roleNames() const
{
    QHash<int, QByteArray> roles;
    roles[to_underlying(Ros2NodeRole::NameRole)] = "name";
    roles[to_underlying(Ros2NodeRole::PackageNameRole)] = "package_name";
    roles[to_underlying(Ros2NodeRole::SubscribersNameRole)] = "subscribers";
    roles[to_underlying(Ros2NodeRole::PublishersNameRole)] = "publishers";
    roles[to_underlying(Ros2NodeRole::DetailInfoRole)] = "detail_info";
    roles[to_underlying(Ros2NodeRole::StateNameRole)] = "state";
    return roles;
}

void Ros2NodeListModel::updateState(std::shared_ptr<Ros2State> state)
{
    if (m_state && !m_state->nodes().empty()) {
        beginRemoveRows(QModelIndex(), 0, m_state->nodes().size() - 1);
        endRemoveRows();
    }
    beginInsertRows(QModelIndex(), 0, state->nodes().size() - 1);
    m_state = std::move(state);
    endInsertRows();
}

QVariant Ros2NodeListModel::getRow(int i, const QString &role_name)
{
    assert(m_string2Role.contains(role_name.toStdString()));
    return data(index(i, 0), to_underlying(m_string2Role[role_name.toStdString()]));
}

QString Ros2NodeListModel::genDetailInfo(const Ros2Node &node) const
{
    QString package_name = "Package name: " + QString(node.package_name.c_str());
    QString subscribers_count = "Subscribers count: " + QString::number(node.subscribers.size());
    QString publishers_count = "Publishers count: " + QString::number(node.publishers.size());
    QString host = "Host: " + QString(node.host.name.c_str()) + ", " + QString(node.host.ip.c_str());
    QString detail_info = package_name + "\n" + subscribers_count + "\n" + publishers_count + "\n" + host;
    return detail_info;
}
QVariant Ros2NodeListModel::getRoleData(const Ros2Node &node, const Ros2NodeRole &role) const
{
    QVariant value;
    if (role == Ros2NodeRole::PackageNameRole) {
        value = QString(node.package_name.c_str());
    } else if (role == Ros2NodeRole::NameRole) {
        value = QString(node.name.c_str());
    } else if (role == Ros2NodeRole::SubscribersNameRole) {
        std::vector<Ros2Subscriber> subscribers = node.subscribers;
        QVector<QString> q_subscribers;
        for (const auto &subscriber: subscribers) {
            q_subscribers.emplace_back(subscriber.topic_name.c_str());
        }
        value = q_subscribers;
    } else if (role == Ros2NodeRole::PublishersNameRole) {
        std::vector<Ros2Publisher> publishers = node.publishers;
        QVector<QString> q_publishers;
        for (const auto &publisher: publishers) {
            q_publishers.emplace_back(publisher.topic_name.c_str());
        }
        value = q_publishers;
    } else if (role == Ros2NodeRole::DetailInfoRole) {
        value = genDetailInfo(node);
    } else if (role == Ros2NodeRole::StateNameRole) {
        value = QString(nodeStatus2Str(node.status).c_str());
    } else if (role == Ros2NodeRole::HostNameRole) {
        value = QString(node.host.ip.c_str());
    }

    return value;
}
}