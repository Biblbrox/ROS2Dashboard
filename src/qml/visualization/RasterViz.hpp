#pragma once

#include "VizComponent.hpp"
#include "qml/models/VisualizerModel.hpp"
#include <QImage>
#include <QtQml/qqmlregistration.h>

namespace ros2monitor {


class RasterViz : public VizComponent {
    Q_OBJECT
    QML_ELEMENT
public:
    explicit RasterViz(QQuickItem *parent = 0);

    void paint(QPainter *painter) override;
    //the paint method is already implemented in QQuickPaintedItem
    //you just override ite

    void updateData(std::any data) override;

public slots:
    void registerViz(VisualizerModel *model, const QString &topic_name, const QString &topic_type);

private:
    QImage m_image;//your image
    std::string m_topic_name;
};
}