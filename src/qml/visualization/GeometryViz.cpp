#include "GeometryViz.hpp"

namespace ros2monitor::viz {

vtkStandardNewMacro(GeometryViz::Data);

void GeometryViz::updateData(std::any data)
{
    m_cloud = std::any_cast<sensor_msgs::msg::PointCloud2>(data);
    m_render_schedule = true;
    emit needRedraw();
}

QQuickVTKItem::vtkUserData GeometryViz::initializeVTK(vtkRenderWindow *renderWindow)
{
    vtkNew<Data> vtk;
    // Create actor
    vtk->actor->SetMapper(vtk->mapper);

    // Create renderer
    vtk->renderer->AddActor(vtk->actor);
    vtk->renderer->SetBackground(0, 0, 0);
    //vtk->renderer->SetBackground2(0.7, 0.7, 0.7);
    vtk->renderer->SetGradientBackground(false);

    renderWindow->AddRenderer(vtk->renderer);
    updatePointCloud(vtk);

    vtk->renderer->GetActiveCamera()->DeepCopy(m_camera);

    return vtk;
}

void GeometryViz::registerViz(VisualizerModel *model, const QString &topic_name, const QString &topic_type)
{
    m_topic_name = topic_name.toStdString();
    model->addTopicViz(VisualizationType::geometry, topic_type.toStdString(), m_topic_name, this);
}

GeometryViz::GeometryViz(QQuickItem *parent) : QQuickVTKItem(parent), m_render_schedule(false)
{
    //connect(this, &QQuickItem::widthChanged, this, &GeometryViz::resetCamera);
    //connect(this, &QQuickItem::heightChanged, this, &GeometryViz::resetCamera);
    connect(this, &GeometryViz::needRedraw, this, [this]() {
        //auto rect = boundingRect().toRect();
        update();
    });
}

void GeometryViz::resetCamera()
{
    dispatch_async([this](vtkRenderWindow *renderWindow, vtkUserData userData) {
        auto *vtk = Data::SafeDownCast(userData);
        vtk->renderer->ResetCamera();
        scheduleRender();
    });
}


/*bool GeometryViz::event(QEvent *ev)
{
    switch (ev->type()) {
        case QEvent::MouseButtonPress: {
            auto e = static_cast<QMouseEvent *>(ev);
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
            _click.reset(new QMouseEvent(e->type(), e->localPos(), e->windowPos(), e->screenPos(),
                                         e->button(), e->buttons(), e->modifiers(), e->source()));
#else
            m_click.reset(e->clone());
#endif
            break;
        }
        case QEvent::MouseMove: {
            if (!m_click)
                return QQuickVTKItem::event(ev);

            auto e = static_cast<QMouseEvent *>(ev);
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
            if ((_click->pos() - e->pos()).manhattanLength() > 5)
#else
            if ((m_click->position() - e->position()).manhattanLength() > 5)
#endif
            {
                QQuickVTKItem::event(QScopedPointer<QMouseEvent>(m_click.take()).get());
                return QQuickVTKItem::event(e);
            }
            break;
        }
        case QEvent::MouseButtonRelease: {
            if (!m_click)
                return QQuickVTKItem::event(ev);
            else
                emit clicked();
            break;
        }
        default:
            break;
    }
    ev->accept();
    return true;
}*/


void GeometryViz::updatePointCloud(vtkUserData userData)
{
    //dispatch_async([this](vtkRenderWindow *renderWindow, vtkUserData userData) {
        // Set points from point cloud to polyData
        vtkNew<vtkPoints> points;
        vtkNew<vtkCellArray> vertices;
        // Fill points with data from m_cloud

        const uint8_t *data = m_cloud.data.data();
        uint32_t point_step = m_cloud.point_step;
        uint32_t width = m_cloud.width;
        for (uint32_t col = 0; col < width; ++col) {
            // Calculate the offset for the current point
            uint32_t index = col * point_step;

            // Access the X, Y, Z coordinates (assuming float32 data type)
            float x = *reinterpret_cast<const float *>(data + index);
            float y = *reinterpret_cast<const float *>(data + index + 4);
            float z = *reinterpret_cast<const float *>(data + index + 8);

            auto pointId = points->InsertNextPoint(x, y, z);
            vertices->InsertNextCell(1);
            vertices->InsertCellPoint(pointId);
        }

        auto *vtk = Data::SafeDownCast(userData);

        vtk->poly_data->SetPoints(points);
        vtk->poly_data->SetVerts(vertices);
        vtk->mapper->SetInputData(vtk->poly_data);
        vtk->mapper->SetColorModeToDefault();
        //vtk->mapper->SetScalarRange(zMin, zMax);
        vtk->mapper->SetScalarVisibility(1);
        //resetCamera();
    //});
}

void GeometryViz::destroyingVTK(vtkRenderWindow *renderWindow, QQuickVTKItem::vtkUserData userData)
{
    auto *vtk = Data::SafeDownCast(userData);
    m_camera->DeepCopy(vtk->renderer->GetActiveCamera());
}
QSGNode *GeometryViz::updatePaintNode(QSGNode *node, QQuickItem::UpdatePaintNodeData *data)
{
    if (m_render_schedule) {
        m_render_schedule = false;
        dispatch_async([this](vtkRenderWindow *renderWindow, vtkUserData userData) {
            updatePointCloud(userData);
            scheduleRender();
        });
    }
    return QQuickVTKItem::updatePaintNode(node, data);
}

}// namespace ros2monitor::viz