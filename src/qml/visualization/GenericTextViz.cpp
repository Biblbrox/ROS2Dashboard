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
    m_text = std::any_cast<const char *>(data);
    this->update(this->contentsBoundingRect().toRect());
}

void GenericTextViz::registerViz(const QString& topic_name, VisualizerModel *model)
{
    m_topic_name = topic_name.toStdString();
    model->addTopicViz(VisualizationType::text, m_topic_name, this);
}

void GenericTextViz::paint(QPainter *painter)
{
    Logger::debug("GenericTextViz");
    VizComponent::paint(painter);
    double x = 0;
    double y = 0;
    QPen pen;
    pen.setColor(QColor::fromRgb(255, 255, 255));
    pen.setWidth(2);
    painter->setPen(pen);
    painter->drawText(x, y, width(), height(), 0, m_text.c_str());
    //painter->drawText(0, 0, 100, 100, 0, m_text.c_str());
}


}