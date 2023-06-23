#include <QPainter>
#include <QQuickItem>

#include "VideoViz.hpp"
#include "core/Logger.hpp"
#include "src/qml/models/VisualizerModel.hpp"

namespace ros2monitor {

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
}

void VideoViz::registerViz(QString topic_name, VisualizerModel *model)
{
    m_topic_name = topic_name.toStdString();
    model->addTopicViz(VisualizationType::image, m_topic_name, this);
}


}