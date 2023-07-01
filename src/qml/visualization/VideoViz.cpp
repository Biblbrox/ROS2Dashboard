#include <QPainter>
#include <QQuickItem>
#include <sensor_msgs/msg/image.hpp>

#include "VideoViz.hpp"
#include "core/Logger.hpp"
#include "src/qml/models/VisualizerModel.hpp"

namespace ros2monitor {

using sensor_msgs::msg::Image;

VideoViz::VideoViz(QQuickItem *parent) : VizComponent(parent)
{
}

void VideoViz::paint(QPainter *painter)
{
    painter->drawImage(QPoint(0, 0), m_image);
}

void VideoViz::updateData(std::any data)
{
    Logger::debug("Receive new image data");

    // Create QImage from sensor_msgs/msg/image
    auto image = std::any_cast<sensor_msgs::msg::Image>(data);
    QImage::Format format;
    if (image.encoding == "mono8")
        format = QImage::Format_Grayscale8;
    else if (image.encoding == "bgr8")
        // There is no BGR format in Qt; B and R channels will be inverted later
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

void VideoViz::registerViz(const QString &topic_name, VisualizerModel *model)
{
    m_topic_name = topic_name.toStdString();
    model->addTopicViz(VisualizationType::raster, "sensor_msgs/msg/Image", m_topic_name, this);
}


}