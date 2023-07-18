#pragma once

#include "VizComponent.hpp"
#include "qml/models/VisualizerModel.hpp"
#include "core/SensorInfo.hpp"

#include <QQuickVTKItem.h>
#include <QVTKRenderWindowAdapter.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vtkActor.h>
#include <vtkBoxWidget.h>
#include <vtkCamera.h>
#include <vtkCommand.h>
#include <vtkConeSource.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkTextActor.h>
#include <vtkTransform.h>

namespace ros2monitor::viz {


struct GeometryViz : QQuickVTKItem {
private:
    Q_OBJECT
    QML_ELEMENT
public:
    explicit GeometryViz(QQuickItem *parent = nullptr);
    vtkUserData initializeVTK(vtkRenderWindow *renderWindow) override;
    void updateData(std::any data);
    //bool event(QEvent *ev) override;
    void resetCamera();
    void destroyingVTK(vtkRenderWindow *renderWindow, QQuickVTKItem::vtkUserData userData) override;
    QSGNode *updatePaintNode(QSGNode *, QQuickItem::UpdatePaintNodeData *) override;
    struct Data : vtkObject {
        static Data *New();
        vtkTypeMacro(Data, vtkObject);

        vtkNew<vtkActor> actor;
        vtkNew<vtkTextActor> text_actor;
        vtkNew<vtkRenderer> renderer;
        vtkNew<vtkPolyDataMapper> mapper;
        vtkNew<vtkPolyData> poly_data;
    };

    Q_SIGNAL void clicked();

public slots:
    void registerViz(ros2monitor::VisualizerModel *model, const QString &topic_name, const QString &topic_type);


signals:
    void needRedraw();

private:
    void updatePointCloud(vtkUserData userData);
    sensor_msgs::msg::PointCloud2 m_cloud;
    vtkNew<vtkCamera> m_camera;
    std::string m_topic_name;
    QScopedPointer<QMouseEvent> m_click;

    std::atomic<bool> m_render_schedule;
    PointCloudSensorInfo m_sensor_data;
};
}