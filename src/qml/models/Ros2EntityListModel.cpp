#include "Ros2EntityListModel.hpp"

#include <utility>

namespace ros2monitor {

    Ros2EntityListModel::Ros2EntityListModel(QObject *parent) : QAbstractListModel(parent) {
        m_role2String[NameRole] = "name";
        m_role2String[ParentNameRole] = "parent_name";
        m_role2String[TypeRole] = "type";
        m_role2String[DetailInfoRole] = "detail_info";

        m_string2Role["name"] = NameRole;
        m_string2Role["parent_name"] = ParentNameRole;
        m_string2Role["type"] = TypeRole;
        m_string2Role["detail_info"] = DetailInfoRole;
    }

    QHash<int, QByteArray> Ros2EntityListModel::roleNames() const {
        QHash<int, QByteArray> roles;
        roles[NameRole] = "name";
        roles[ParentNameRole] = "parent_name";
        roles[TypeRole] = "type";
        roles[DetailInfoRole] = "detail_info";
        return roles;
    }

    void Ros2EntityListModel::updateState(std::shared_ptr<Ros2State> state) {
        beginResetModel();
        m_state = std::move(state);
        endResetModel();
    }

    QVariant Ros2EntityListModel::getRow(int i, QString role_name) {
        assert(m_string2Role.contains(role_name.toStdString()));
        return data(index(i, 0), m_string2Role[role_name.toStdString()]);
    }

}