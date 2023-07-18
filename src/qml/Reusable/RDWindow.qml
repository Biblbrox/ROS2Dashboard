import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Shapes
import QtQuick.Window
import com.viz.types 1.0

Window {
    id: floatingWindow

    function load(componentFile, topicName, topicType) {
        Logger.debug("Loading component: " + componentFile + " with topic: " + topicName + " of type: " + topicType);
        vizLoader.active = true;
        vizLoader.setSource(componentFile, {
                "topicName": topicName,
                "topicType": topicType,
                "width": parent.width,
                "height": parent.height
            });
    }

    Loader {
        id: vizLoader

        active: false
        height: parent.height
        width: parent.width
    }
}