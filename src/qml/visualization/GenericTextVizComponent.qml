import QtQuick
import com.viz.types 1.0

GenericTextViz {
    id: textViz

    property string topicName
    property string topicType

    //anchors.fill: parent
    height: parent.height
    width: parent.width

    Component.onCompleted: {
        Logger.debug("GenericTextViz created for topic " + topicName + " with type " + topicType);
        //if (!visualizerModel.hasTopicViz(topicName)) {
            textViz.registerViz(visualizerModel, topicName, topicType);
        //}
    }
}
