import QtQuick
import com.viz.types 1.0

Item {
    property string topicName

    GenericTextViz {
        id: textViz

        anchors.fill: parent
        height: parent.height
        width: parent.width

        Component.onCompleted: {
            console.log("GenericTextViz created for topic " + topicName);
            textViz.registerViz(topicName, visualizerModel);
        }
    }
}