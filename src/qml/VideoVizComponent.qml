import QtQuick
import com.viz.types 1.0

Item {
    property string topicName
    property string topicType

    VideoViz {
        id: videoViz

        anchors.fill: parent
        height: parent.height
        width: parent.width

        Component.onCompleted: {
            console.log("VideoViz created for topic " + topicName + " with type " + topicType);
            videoViz.registerViz(visualizerModel, topicName, topicType);
        }
    }
}