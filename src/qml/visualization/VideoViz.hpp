#pragma once

#include "VizComponent.hpp"
#include "qml/models/VisualizerModel.hpp"
#include <QImage>
#include <QtQml/qqmlregistration.h>

namespace ros2monitor {


class VideoViz : public VizComponent {
    Q_OBJECT
    QML_ELEMENT
public:
    explicit VideoViz(QQuickItem *parent = 0);

    void paint(QPainter *painter) override;
    //the paint method is already implemented in QQuickPaintedItem
    //you just override ite

    void updateData(std::any data) override;

public slots:
    void registerViz(QString topic_name, VisualizerModel *model);

private:
    QImage m_image;//your image
    std::string m_topic_name;
};
}