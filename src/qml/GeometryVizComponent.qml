import QtQuick
import com.viz.types 1.0
import com.vtk.types 1.0

GeometryViz {
    id: geometryViz

    property string topicName
    property string topicType

    height: parent.height
    width: parent.width
    Component.onCompleted: {
        Logger.debug("GeometryViz created for topic " + topicName + " with type " + topicType);
        geometryViz.registerViz(visualizerModel, topicName, topicType);
    }
}
