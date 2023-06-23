#pragma once

#include <QQuickPaintedItem>
#include <any>
namespace ros2monitor {


class VizComponent : public QQuickPaintedItem {
    Q_OBJECT
    //QML_ELEMENT
public:
    explicit VizComponent(QQuickItem* parent = nullptr) : QQuickPaintedItem(parent) {}
    ~VizComponent() override = default;
    virtual void updateData(std::any data) = 0;
    void paint(QPainter *painter) override;
};

}