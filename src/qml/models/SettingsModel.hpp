#pragma once

#include "core/Config.hpp"
#include <QAbstractListModel>

namespace ros2monitor {

enum class SettingsRole {
    GroupRole = Qt::UserRole + 1,
    NameRole = Qt::UserRole + 2,
    ValueRole = Qt::UserRole + 3,
};

struct Key {
    std::string group;
    std::string name;

    Key(std::string group, std::string name) : group(std::move(group)), name(std::move(name)) { }

    [[nodiscard]] std::string concat() const
    {
        return group + "." + name;
    }
};

class SettingsModel : public QAbstractListModel {

    Q_OBJECT
public:
    // Pair of parameter name and its type
    using Parameter = std::pair<std::string, std::string>;// (value, type)
    using Parameters = std::unordered_map<std::string, Parameter>;// (name, parameter)
    using Group = std::unordered_map<std::string, Parameters>;

    explicit SettingsModel(std::shared_ptr<Config> config = nullptr, QObject *parent = nullptr);
    ~SettingsModel() override;
    QHash<int, QByteArray> roleNames() const override;
    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;

public slots:
    QVariant getValue(const QString &group_name, const QString &param_name) const;
    QVariant getGroupParams(const QString &group_name) const;

private:
    std::pair<bool, std::string> validateConfig();
    QString getConfigValue(const std::string &group_name, const std::string param_name) const;

    std::shared_ptr<Config> m_config;
    std::vector<Key> m_keys;
    /**
     * Map of available parameters in format (group name, parameter)
     */
    Group m_groups;
    std::vector<std::string> m_group_names;
};

}// namespace ros2monitor
