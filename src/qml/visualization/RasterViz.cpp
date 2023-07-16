#include <QPainter>
#include <QQuickItem>
#include <sensor_msgs/msg/image.hpp>

#include "RasterViz.hpp"
#include "core/Logger.hpp"
#include "src/qml/models/VisualizerModel.hpp"

namespace ros2monitor::viz {

using sensor_msgs::msg::Image;

RasterViz::RasterViz(QQuickItem *parent) : VizComponent(parent)
{
}

void RasterViz::paint(QPainter *painter)
{
    VizComponent::paint(painter);
    QImage resized = m_image.scaled(width(), height(), Qt::KeepAspectRatio);
    painter->drawImage(QPoint(0, 0), resized);
}

void RasterViz::updateData(std::any data)
{
    // Create QImage from sensor_msgs/msg/image
    auto image = std::any_cast<sensor_msgs::msg::Image>(data);
    QImage::Format format;
    if (image.encoding == "mono8")
        format = QImage::Format_Grayscale8;
    else if (image.encoding == "bgr8")
        format = QImage::Format_BGR888;
    else if (image.encoding == "rgb8")
        format = QImage::Format_RGB888;
    else {
        Logger::error("Image pixel format not supported!");
        return;
    }

    m_image = QImage(&image.data[0], static_cast<int>(image.width), static_cast<int>(image.height), format);
    emit needRedraw();
}

void RasterViz::registerViz(VisualizerModel *model, const QString &topic_name, const QString &topic_type)
{
    m_topic_name = topic_name.toStdString();
    model->addTopicViz(VisualizationType::raster, topic_type.toStdString(), m_topic_name, this);
}


}