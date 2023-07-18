import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import com.viz.types 1.0
import com.vtk.types 1.0

GeometryViz {
    id: geometryViz

    property string topicName
    property string topicType

    property string stats: ""

    height: parent.height
    width: parent.width

    Label {
        id: statsLabel
        text: stats
        width: parent.width / 3
        height: parent.height / 3
    }

    Component.onCompleted: {
        Logger.debug("GeometryViz created for topic " + topicName + " with type " + topicType);
        //if (!visualizerModel.hasTopicViz(topicName)) {
            geometryViz.registerViz(visualizerModel, topicName, topicType);
        //}
    }
}
