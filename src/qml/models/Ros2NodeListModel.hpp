#pragma once

#include "ros2_entities/Ros2State.hpp"
#include <QAbstractListModel>

namespace ros2monitor {

enum class Ros2NodeRole {
    NameRole = Qt::UserRole + 1,
    PackageNameRole,
    DetailInfoRole,
    SubscribersNameRole,
    PublishersNameRole
};

class Ros2NodeListModel : public QAbstractListModel {
    Q_OBJECT
public:
    explicit Ros2NodeListModel(QObject *parent = nullptr);

    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;

    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    QHash<int, QByteArray> roleNames() const override;
    void updateState(std::shared_ptr<Ros2State> state);
public slots:
    QVariant getRow(int i, const QString& role_name);
    QVariant getRowByName(int i, const QString& role_name, const QString& entry_name);

private:
    std::shared_ptr<Ros2State> m_state;
    std::unordered_map<Ros2NodeRole, std::string> m_role2String;
    std::unordered_map<std::string, Ros2NodeRole> m_string2Role;
};
}
