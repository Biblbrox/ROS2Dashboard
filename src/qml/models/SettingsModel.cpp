#include "SettingsModel.hpp"
#include "core/Utils.hpp"

namespace ros2monitor {

SettingsModel::SettingsModel(std::shared_ptr<Config> config, QObject *parent) : QAbstractListModel(parent), m_config(std::move(config))
{
}

QHash<int, QByteArray> SettingsModel::roleNames() const
{
    QHash<int, QByteArray> roles;
    roles[to_underlying(SettingsRole::NameRole)] = "name";
    return roles;
}

int SettingsModel::rowCount(const QModelIndex &parent) const
{
    return m_config->size();
}

QVariant SettingsModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid()) {
        return QVariant();
    }

    if (role == to_underlying(SettingsRole::NameRole)) {
        //return QString::fromStdString(m_config->getByIdx<>(index.row()).first);
    }

    return QVariant();
}
SettingsModel::~SettingsModel()
{
}

}// namespace ros2monitor
