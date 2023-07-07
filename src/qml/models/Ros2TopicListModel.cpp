#include "Ros2TopicListModel.hpp"

namespace ros2monitor {

Ros2TopicListModel::Ros2TopicListModel(QObject *parent) : Ros2EntityListModel(parent)
{
}
QVariant Ros2TopicListModel::data(const QModelIndex &index, int role) const
{
    if (!m_state || !index.isValid())
        return QVariant{};
    QVariant value;
    if (role == ParentNameRole) {
        value = QString(m_state->topics()[index.row()].node_name.c_str());
    } else if (role == NameRole) {
        value = QString(m_state->topics()[index.row()].name.c_str());
    } else if (role == TypeRole) {
        value = QString(m_state->topics()[index.row()].type.c_str());
    }
    return value;
}

int Ros2TopicListModel::rowCount(const QModelIndex &parent) const
{
    return m_state->topics().size();
}

QVariant Ros2TopicListModel::getRowByName(int i, QString role_name, QString entry_name)
{
    return QVariant();
}

}