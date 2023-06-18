#pragma once

#include "Ros2EntityListModel.hpp"

namespace ros2monitor {
    class Ros2PackageListModel : public Ros2EntityListModel {
    public:
        explicit Ros2PackageListModel(QObject *parent = nullptr);
        QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
        int rowCount(const QModelIndex &parent = QModelIndex()) const override;
        QVariant getRowByName(int i, QString role_name, QString entry_name) override;
    };
}
