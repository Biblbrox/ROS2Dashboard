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

void GenericTextViz::registerViz(const QString &topic_name, VisualizerModel *model)
{
    m_topic_name = topic_name.toStdString();
    model->addTopicViz(VisualizationType::text, m_topic_name, this);
}

void GenericTextViz::paint(QPainter *painter)
{
    VizComponent::paint(painter);
    QPen pen;
    pen.setColor(QColor::fromRgb(255, 255, 255));
    pen.setWidth(2);
    QFont font("sans", 22);

    QRect br = painter->boundingRect(0, 0, width(), height(), 0, m_text.c_str());
    painter->setFont(font);
    painter->setPen(pen);
    painter->drawText(width() / 2 - br.width() / 2, height() / 2 - br.height() / 2, width(), height(), 0, m_text.c_str());
}


}