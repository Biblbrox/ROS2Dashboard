#pragma once

#include <QAbstractListModel>

#include "ros2_entities/Ros2State.hpp"

namespace ros2monitor {

    enum Ros2EntityRole {
        NameRole = Qt::UserRole + 1,
        ParentNameRole,
        TypeRole
    };

    class Ros2EntityListModel : public QAbstractListModel {
    Q_OBJECT

    public:
        explicit Ros2EntityListModel(QObject *parent = nullptr);

        QHash<int, QByteArray> roleNames() const override;

        void updateState(std::shared_ptr<Ros2State> state);

    public slots:

        virtual QVariant getRow(int i, QString role_name);
        virtual QVariant getRowByName(int i, QString role_name, QString entry_name) = 0;

    protected:
        std::shared_ptr<Ros2State> m_state;
        std::unordered_map<Ros2EntityRole, std::string> m_role2String;
        std::unordered_map<std::string, Ros2EntityRole> m_string2Role;
    };
}