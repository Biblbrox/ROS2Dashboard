import QtQuick
import com.viz.types 1.0

RasterViz {
    id: videoViz

    property string topicName
    property string topicType

    //anchors.fill: parent
    height: parent.height
    width: parent.width

    Component.onCompleted: {
        Logger.debug("VideoViz created for topic " + topicName + " with type " + topicType);
        //if (!visualizerModel.hasTopicViz(topicName)) {
            videoViz.registerViz(visualizerModel, topicName, topicType);
        //}
    }
}
