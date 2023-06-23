import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtWebEngine
import com.viz.types 1.0

Rectangle {
    id: visualizationWindow

    color: "#2f2e40"

    /*WebEngineView {
        url: "http://localhost:8888/"
        anchors {
            bottom: parent.bottom
            left: parent.left
            right: parent.right
            top: delimeter.bottom
        }
    }*/

    Component.onCompleted: {
        console.log("Visualization area loaded");
        textViz.registerViz("minimal_publisher", visualizerModel);
    }

    Rectangle {
        id: delimeter

        color: "#3d3948"
        height: 30
        width: parent.width

        anchors {
            left: parent.left
            right: parent.right
            top: parent.top
        }
    }
    GenericTextViz {
        id: textViz

        height: 200
        width: 200
    }
}
