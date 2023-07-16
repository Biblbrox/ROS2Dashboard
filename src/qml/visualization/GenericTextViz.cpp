#include <QPainter>
#include <QQuickPaintedItem>

#include "GenericTextViz.hpp"
#include "core/Logger.hpp"

namespace ros2monitor::viz {

GenericTextViz::GenericTextViz(QQuickItem *parent) : VizComponent(parent)
{
}

void GenericTextViz::updateData(std::any data)
{
    m_text = std::any_cast<std::string>(data);

    emit needRedraw();
}

void GenericTextViz::registerViz(VisualizerModel *model, const QString &topic_name, const QString &topic_type)
{
    m_topic_name = topic_name.toStdString();
    model->addTopicViz(VisualizationType::string, topic_type.toStdString(), m_topic_name, this);
}

void GenericTextViz::paint(QPainter *painter)
{
    VizComponent::paint(painter);
    QPen pen;
    pen.setColor(QColor::fromRgb(255, 255, 255));
    pen.setWidth(2);
    QFont font("sans", 22);

    painter->setFont(font);
    painter->setPen(pen);
    painter->drawText(0, 0, width(), height(), 0, m_text.c_str());

    QQuickPaintedItem::update(boundingRect().toRect());
}


}