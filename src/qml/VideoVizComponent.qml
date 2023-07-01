import QtQuick
import com.viz.types 1.0

Item {
    property string topicName

    VideoViz {
        id: videoViz

        anchors.fill: parent
        height: parent.height
        width: parent.width

        Component.onCompleted: {
            console.log("VideoViz created for topic " + topicName);
            videoViz.registerViz(topicName, visualizerModel);
        }
    }
}