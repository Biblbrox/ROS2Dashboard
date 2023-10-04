#include <fmt/format.h>

#include "SettingsModel.hpp"
#include "core/Utils.hpp"

namespace ros2monitor {

using fmt::format;

SettingsModel::SettingsModel(std::shared_ptr<Config> config, QObject *parent) : QAbstractListModel(parent), m_config(std::move(config))
{

    Parameters daemon_params = { { "daemon_pid_file", { "/tmp/ros2monitor.sock", "string" } }, { "ros_domain_id", { "1", "int" } } };
    Parameters client_params = { { "enable_utility_topics", { "true", "bool" } }, { "enable_utility_nodes", { "true", "bool" } } };
    m_groups = { { "daemon", daemon_params }, { "client", client_params } };

    for (const auto &kv: m_groups) {
        m_group_names.push_back(kv.first);
    }

    auto validated = validateConfig();
    if (!validated.first) {
        throw std::runtime_error(validated.second);
    }
}

QHash<int, QByteArray> SettingsModel::roleNames() const
{
    QHash<int, QByteArray> roles;
    roles[to_underlying(SettingsRole::NameRole)] = "name";
    roles[to_underlying(SettingsRole::GroupRole)] = "group";
    roles[to_underlying(SettingsRole::ValueRole)] = "value";
    return roles;
}

int SettingsModel::rowCount(const QModelIndex &parent) const
{
    return m_groups.size();
}

QVariant SettingsModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid()) {
        return {};
    }

    int idx = index.row();
    if (role == to_underlying(SettingsRole::GroupRole)) {
        return QString::fromStdString(m_group_names[idx]);
    }

    return {};
}

SettingsModel::~SettingsModel()
{
}

std::pair<bool, std::string> SettingsModel::validateConfig()
{
    // Check if config contains all parameters according to m_parameters.
    for (const auto &[group_name, parameters]: m_groups) {
        if (m_config->find(group_name) == m_config->end()) {
            return { false, "Config file does not contain group name: " + group_name };
        }

        // Check all the parameters in group group_name
        for (const auto &[parameter_name, parameter_type]: parameters) {
            std::string key = group_name;
            key += ".";
            key += parameter_name;
            if (!m_config->contains(key)) {
                return { false, format("Config file does not contain parameter with name {} in group {}", key, group_name) };
            }

            m_keys.emplace_back(group_name, parameter_name);
        }
    }

    return { true, "" };
}

QVariant SettingsModel::getConfigValue(const QString &group_name, const QString &param_name) const
{
    QString key = group_name;
    key += ".";
    key += param_name;

    if (m_groups.at(group_name.toStdString()).at(param_name.toStdString()).second == "int") {
        return m_config->get<int>(key.toStdString());
    } else if (m_groups.at(group_name.toStdString()).at(param_name.toStdString()).second == "string") {
        return QString::fromStdString(m_config->get<std::string>(key.toStdString()));
    } else if (m_groups.at(group_name.toStdString()).at(param_name.toStdString()).second == "bool") {
        return m_config->get<bool>(key.toStdString());
    }
}

QVariant SettingsModel::getValue(const QString &group_name, const QString &param_name) const
{
    if (m_groups.find(group_name.toStdString()) == m_groups.end()) {
        throw std::invalid_argument("Group name does not exist: " + group_name.toStdString());
    }

    return getConfigValue(group_name, param_name);
}

QVariant SettingsModel::getGroupParams(const QString &group_name) const
{
    if (m_groups.find(group_name.toStdString()) == m_groups.end()) {
        throw std::invalid_argument("Group name does not exist: " + group_name.toStdString());
    }

    QVariantList params;
    for (const auto &[param_name, param_type]: m_groups.at(group_name.toStdString())) {
        QVariantMap param;
        param["name"] = QString::fromStdString(param_name);
        param["value"] = getConfigValue(group_name, QString::fromStdString(param_name));
        params.push_back(param);
    }

    return params;
}

void SettingsModel::setConfigValue(const QString &group_name, const QString &param_name, const QString &value, const QString &type)
{
    QString key = group_name;
    key += ".";
    key += param_name;

    if (type == "int") {
        m_config->set(key.toStdString(), std::stoi(value.toStdString()));
    } else if (type == "string") {
        m_config->set(key.toStdString(), value.toStdString());
    } else if (type == "bool") {
        m_config->set(key.toStdString(), value == "true");
    }

    m_config->save(m_config->path());
}

}// namespace ros2monitor
