#include <QPainter>
#include <QQuickPaintedItem>

#include "GenericTextViz.hpp"
#include "core/Logger.hpp"

namespace ros2monitor {

GenericTextViz::GenericTextViz(QQuickItem *parent) : VizComponent(parent)
{
}

void GenericTextViz::updateData(std::any data)
{
    Logger::debug("Receive new text data");
    m_text = std::any_cast<std::string>(data);
}

void GenericTextViz::registerViz(QString topic_name, VisualizerModel *model)
{
    m_topic_name = topic_name.toStdString();
    model->addTopicViz(VisualizationType::text, m_topic_name, this);
}

void GenericTextViz::paint(QPainter *painter)
{
    Logger::debug("GenericTextViz");
    VizComponent::paint(painter);
    painter->drawText(0, 0, m_text.c_str());
}


}