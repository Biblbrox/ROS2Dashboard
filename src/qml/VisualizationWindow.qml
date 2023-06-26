import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import com.viz.types 1.0

Rectangle {
    id: visualizationWindow

    color: "#2f2e40"

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
    SplitView {
        id: splitView

        height: parent.height
        orientation: Qt.Horizontal
        width: parent.width

        anchors {
            bottom: parent.bottom
            left: parent.left
            right: parent.right
            top: delimeter.bottom
        }
        ListView {
            id: topicList

            Layout.fillHeight: true
            clip: true
            height: parent.height
            implicitWidth: 600
            model: topicListModel
            spacing: 10

            delegate: Item {
                height: 70
                width: topicList.width

                Rectangle {
                    //clip: true
                    color: Theme.panel.color.elementBackground
                    height: parent.height
                    radius: 10
                    width: parent.width

                    TextArea {
                        id: topicName

                        color: Theme.font.color.primary
                        font.pointSize: 18
                        height: parent.height
                        horizontalAlignment: TextEdit.AlignHCenter
                        readOnly: true
                        text: name
                        verticalAlignment: TextEdit.AlignVCenter
                        width: parent.width
                    }
                    MouseArea {
                        acceptedButtons: Qt.LeftButton
                        anchors.fill: parent

                        onClicked: {
                            console.debug("Clicked to element " + name);
                        }
                    }
                }
            }
        }
        GenericTextViz {
            id: textViz

            Layout.fillHeight: true
            SplitView.fillWidth: true
            height: parent.height
        }
    }
}
