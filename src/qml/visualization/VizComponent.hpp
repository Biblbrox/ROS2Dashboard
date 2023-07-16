#pragma once

#include <QQuickPaintedItem>
#include <any>

#include "core/Logger.hpp"
namespace ros2monitor::viz {


class VizComponent : public QQuickPaintedItem {
    Q_OBJECT
    //QML_ELEMENT
public:
    explicit VizComponent(QQuickItem *parent = nullptr) : QQuickPaintedItem(parent)
    {
        connect(this, &VizComponent::needRedraw, this, [this]() {
            auto rect = boundingRect().toRect();
            update(rect);
        });
    }
    ~VizComponent() override = default;
    virtual void updateData(std::any data) = 0;
    void paint(QPainter *painter) override;

signals:
    void needRedraw();
};

}