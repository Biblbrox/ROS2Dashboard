#pragma once

#include "core/Config.hpp"
#include <QAbstractListModel>

namespace ros2monitor {

enum class SettingsRole {
    NameRole = Qt::UserRole + 1,
};

class SettingsModel : public QAbstractListModel {

    Q_OBJECT
public:
    explicit SettingsModel(std::shared_ptr<Config> config = nullptr, QObject *parent = nullptr);
    ~SettingsModel() override;
    QHash<int, QByteArray> roleNames() const override;
    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;

private:
    std::shared_ptr<Config> m_config;
};

}// namespace ros2monitor
