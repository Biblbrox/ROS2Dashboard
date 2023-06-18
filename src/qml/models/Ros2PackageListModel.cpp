#include "Ros2PackageListModel.hpp"

namespace ros2monitor {
    Ros2PackageListModel::Ros2PackageListModel(QObject *parent) : Ros2EntityListModel(parent) {
    }

    QVariant Ros2PackageListModel::data(const QModelIndex &index, int role) const {
        if (!m_state || !index.isValid())
            return QVariant{};

        QVariant value;
        if (role == ParentNameRole) {
            value = m_state->topics()[index.row()].node_name.c_str();
        } else if (role == NameRole) {
            value = m_state->topics()[index.row()].name.c_str();
        } else if (role == TypeRole) {
            value = "package";
        }

        return value;
    }

    int Ros2PackageListModel::rowCount(const QModelIndex &parent) const {
        return m_state ? m_state->topics().size() : 0;
    }

    QVariant Ros2PackageListModel::getRowByName(int i, QString role_name, QString entry_name) {
        assert(m_string2Role.contains(role_name.toStdString()));
        Ros2EntityRole role = m_string2Role[role_name.toStdString()];
        QVariant value;
        if (role == ParentNameRole) {
            Ros2Package package = m_state->package(entry_name.toLocal8Bit().constData());
            value = QString(package.name.c_str());
        } else if (role == NameRole) {
            Ros2Package package = m_state->package(entry_name.toLocal8Bit().constData());
            value = QString(package.name.c_str());
        } else if (role == TypeRole) {
            value = QString("package");
        }

        return value;
    }
}
