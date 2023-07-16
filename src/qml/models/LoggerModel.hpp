#pragma once

#include "core/Logger.hpp"
#include <QObject>
#include <QtQmlIntegration>

namespace ros2monitor {

class LoggerModel : public QObject {
    Q_OBJECT
    QML_NAMED_ELEMENT(Logger)
    QML_SINGLETON
public:
    explicit LoggerModel(QObject *parent = nullptr) : QObject(parent)
    {
    }

public slots:
    static void debug(const QString &message)
    {
        Logger::debug(message.toStdString());
    }

    static void info(const QString &message)
    {
        Logger::info(message.toStdString());
    }

    static void warn(const QString &message)
    {
        Logger::warn(message.toStdString());
    }

    static void error(const QString &message)
    {
        Logger::error(message.toStdString());
    }
};
}// namespace ros2monitor