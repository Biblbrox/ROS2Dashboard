#include "Ros2NodeListModel.hpp"

#include "core/Logger.hpp"
#include <utility>


namespace ros2monitor {

template<typename E>
constexpr auto to_underlying(E e) noexcept
{
    return static_cast<std::underlying_type_t<E>>(e);
}


Ros2NodeListModel::Ros2NodeListModel(QObject *parent) : QAbstractListModel(parent)
{
    m_role2String[Ros2NodeRole::NameRole] = "name";
    m_role2String[Ros2NodeRole::PackageNameRole] = "package_name";
    m_role2String[Ros2NodeRole::SubscribersNameRole] = "subscribers";
    m_role2String[Ros2NodeRole::PublishersNameRole] = "publishers";

    m_string2Role["name"] = Ros2NodeRole::NameRole;
    m_string2Role["package_name"] = Ros2NodeRole::PackageNameRole;
    m_string2Role["subscribers"] = Ros2NodeRole::SubscribersNameRole;
    m_string2Role["publishers"] = Ros2NodeRole::PublishersNameRole;
}

QVariant Ros2NodeListModel::data(const QModelIndex &index, int role) const
{
    if (!m_state || !index.isValid())
        return QVariant{};
    QVariant value;
    if (role == to_underlying(Ros2NodeRole::PackageNameRole)) {
        value = QString(m_state->nodes()[index.row()].package_name.c_str());
    } else if (role == to_underlying(Ros2NodeRole::NameRole)) {
        value = QString(m_state->nodes()[index.row()].name.c_str());
    } else if (role == to_underlying(Ros2NodeRole::SubscribersNameRole)) {
        std::vector<Ros2Subscriber> subscribers = m_state->nodes()[index.row()].subscribers;
        QVector<QString> q_subscribers;
        for (const auto &subscriber: subscribers) {
            q_subscribers.emplace_back(subscriber.topic_name.c_str());
        }
        value = q_subscribers;
    } else if (role == to_underlying(Ros2NodeRole::PublishersNameRole)) {
        std::vector<Ros2Publisher> publishers = m_state->nodes()[index.row()].publishers;
        QVector<QString> q_publishers;
        for (const auto &publisher: publishers) {
            q_publishers.emplace_back(publisher.topic_name.c_str());
        }
        value = q_publishers;
    }
    return value;
}

int Ros2NodeListModel::rowCount(const QModelIndex &parent) const
{
    return m_state ? m_state->nodes().size() : 0;
}

QVariant Ros2NodeListModel::getRowByName(int i, QString role_name, QString entry_name)
{
    assert(m_string2Role.contains(role_name.toStdString()));
    Ros2NodeRole role = m_string2Role[role_name.toStdString()];
    QVariant value;
    if (role == Ros2NodeRole::PackageNameRole) {
        Ros2Node node = m_state->node(entry_name.toLocal8Bit().constData());
        value = QString(node.package_name.c_str());
    } else if (role == Ros2NodeRole::NameRole) {
        Ros2Node node = m_state->node(entry_name.toLocal8Bit().constData());
        value = QString(node.name.c_str());
    } else if (role == Ros2NodeRole::SubscribersNameRole) {
        std::vector<Ros2Subscriber> subscribers = m_state->node(entry_name.toLocal8Bit().constData()).subscribers;
        QVector<QString> q_subscribers;
        for (const auto &subscriber: subscribers) {
            q_subscribers.emplace_back(subscriber.topic_name.c_str());
        }
        value = q_subscribers;
    } else if (role == Ros2NodeRole::PublishersNameRole) {
        std::vector<Ros2Publisher> publishers = m_state->node(entry_name.toLocal8Bit().constData()).publishers;
        QVector<QString> q_publishers;
        for (const auto &publisher: publishers) {
            q_publishers.emplace_back(publisher.topic_name.c_str());
        }
        value = q_publishers;
    }

    return value;
}

QHash<int, QByteArray> Ros2NodeListModel::roleNames() const
{
    QHash<int, QByteArray> roles;
    roles[to_underlying(Ros2NodeRole::NameRole)] = "name";
    roles[to_underlying(Ros2NodeRole::PackageNameRole)] = "package_name";
    roles[to_underlying(Ros2NodeRole::SubscribersNameRole)] = "subscribers";
    roles[to_underlying(Ros2NodeRole::PublishersNameRole)] = "publishers";
    return roles;
}

void Ros2NodeListModel::updateState(std::shared_ptr<Ros2State> state)
{
    m_state = std::move(state);
}

QVariant Ros2NodeListModel::getRow(int i, QString role_name)
{
    assert(m_string2Role.contains(role_name.toStdString()));
    return data(index(i, 0), to_underlying(m_string2Role[role_name.toStdString()]));
}
}