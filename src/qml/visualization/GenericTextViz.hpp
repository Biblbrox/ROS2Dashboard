#pragma once

#include "VizComponent.hpp"
#include "src/qml/models/VisualizerModel.hpp"
#include <QQuickPaintedItem>
#include <QtQml/qqmlregistration.h>

namespace ros2monitor::viz {

class GenericTextViz : public VizComponent {
    Q_OBJECT
    QML_ELEMENT
public:
    explicit GenericTextViz(QQuickItem *parent = 0);

    void updateData(std::any data) override;
    void paint(QPainter *painter) override;

public slots:
    void registerViz(VisualizerModel *model, const QString &topic_name, const QString &topic_type);

private:
    std::string m_text;
    std::string m_topic_name;

};

}