#include "Ros2NodeListModel.hpp"
#include "core/Logger.hpp"


namespace ros2monitor {

    Ros2NodeListModel::Ros2NodeListModel(QObject *parent) : Ros2EntityListModel(parent) {
    }

    QVariant Ros2NodeListModel::data(const QModelIndex &index, int role) const {
        if (!m_state || !index.isValid())
            return QVariant{};
        QVariant value;
        if (role == ParentNameRole) {
            value = QString(m_state->nodes()[index.row()].package_name.c_str());
        } else if (role == NameRole) {
            value = QString(m_state->nodes()[index.row()].name.c_str());
        } else if (role == TypeRole) {
            value = QString("node");
        }
        return value;
    }

    int Ros2NodeListModel::rowCount(const QModelIndex &parent) const {
        return m_state ? m_state->nodes().size() : 0;
    }

    QVariant Ros2NodeListModel::getRowByName(int i, QString role_name, QString entry_name) {
        assert(m_string2Role.contains(role_name.toStdString()));
        Ros2EntityRole role = m_string2Role[role_name.toStdString()];
        QVariant value;
        if (role == ParentNameRole) {
            Ros2Node node = m_state->node(entry_name.toLocal8Bit().constData());
            value = QString(node.package_name.c_str());
        } else if (role == NameRole) {
            Ros2Node node = m_state->node(entry_name.toLocal8Bit().constData());
            value = QString(node.name.c_str());
        } else if (role == TypeRole) {
            value = QString("node");
        }

        return value;
    }
}