#include "Ros2TopicListModel.hpp"
#include "core/Logger.hpp"


namespace ros2monitor {

    Ros2TopicListModel::Ros2TopicListModel(QObject *parent) : Ros2EntityListModel(parent) {
    }

    QVariant Ros2TopicListModel::data(const QModelIndex &index, int role) const {
        if (!m_state || !index.isValid())
            return QVariant{};
        QVariant value;
        if (role == ParentNameRole) {
            value = QString(m_state->topics()[index.row()].node_name.c_str());
        } else if (role == NameRole) {
            value = QString(m_state->topics()[index.row()].name.c_str());
        } else if (role == TypeRole) {
            value = QString("topic");
        }
        return value;
    }

    int Ros2TopicListModel::rowCount(const QModelIndex &parent) const {
        return m_state ? m_state->topics().size() : 0;
    }

    QVariant Ros2TopicListModel::getRowByName(int i, QString role_name, QString entry_name) {
        assert(m_string2Role.contains(role_name.toStdString()));
        Ros2EntityRole role = m_string2Role[role_name.toStdString()];
        QVariant value;
        if (role == ParentNameRole) {
            Ros2Topic topic = m_state->topic(entry_name.toLocal8Bit().constData());
            value = QString(topic.node_name.c_str());
        } else if (role == NameRole) {
            Ros2Topic topic = m_state->topic(entry_name.toLocal8Bit().constData());
            value = QString(topic.name.c_str());
        } else if (role == TypeRole) {
            value = QString("topic");
        }

        return value;

    }
}