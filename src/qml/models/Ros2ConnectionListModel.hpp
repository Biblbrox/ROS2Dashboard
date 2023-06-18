#pragma once

#include "Ros2EntityListModel.hpp"

namespace ros2monitor {


    enum Ros2ConnectionRole {
        TopicNameRole = Qt::UserRole + 1,
        SrcNodeNameRole,
        DstNodeNameRole,
    };

    class Ros2ConnectionListModel : public QAbstractListModel {
    Q_OBJECT

    public:
        explicit Ros2ConnectionListModel(QObject *parent = nullptr);

        QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;

        void update(std::vector<Ros2Connection> connections);

        QHash<int, QByteArray> roleNames() const override;

        int rowCount(const QModelIndex &parent = QModelIndex()) const override;

    public slots:
        QVariant getRow(int i, const QString &role_name);

    private:
        std::vector<Ros2Connection> m_connections;
        std::unordered_map<std::string, Ros2ConnectionRole> m_string2Role;
    };
}
