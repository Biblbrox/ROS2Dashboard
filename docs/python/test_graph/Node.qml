import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts
import QtQuick.Shapes 1.15

Rectangle {
    id: rect

    property int sceneWidth
    property alias color: rect.color
    property alias name: nodeLabel.text

    signal outputPortClicked(string topicName)

    radius: 10
    //width: width
    //height: max(outputPorts.height, inputPorts.height, height)
    Layout.minimumWidth: width
    Layout.minimumHeight: max(outputPorts.height, inputPorts.height, height)

    ColumnLayout {
        anchors.fill: parent

        Label {
            id: nodeLabel

            width: parent.width
            Layout.rightMargin: 20
            Layout.leftMargin: 20
            Layout.fillWidth: true
        }

    }

    ColumnLayout {
        id: inputPorts

        anchors {
            left: parent.left
            top: parent.top
            bottom: parent.bottom
        }

        Port {
            input: true
        }

    }

    ColumnLayout {
        id: outputPorts

        anchors {
            right: parent.right
            top: parent.top
            bottom: parent.bottom
        }

        Port {
            input: false
            topicName: "turtlesim"
            onPortClicked: function(isInput, topicName) {
                console.log(isInput);
                if (!isInput) {
                    console.log('Port clicked, ', isInput, ", ", topicName);
                    rect.outputPortClicked(topicName);
                }
            }
        }

        Port {
            input: false
            onPortClicked: function(isInput, topicName) {
                console.log(isInput);
                if (!isInput) {
                    console.log('Port clicked, ', isInput, ", ", topicName);
                    rect.outputPortClicked(topicName);
                }
            }
        }

        Port {
            input: false
            onPortClicked: function(isInput, topicName) {
                console.log(isInput);
                if (!isInput) {
                    console.log('Port clicked, ', isInput, ", ", topicName);
                    rect.outputPortClicked(topicName);
                }
            }
        }

    }

    MouseArea {
        //drag.minimumX: 0
        //drag.maximumX: sceneWidth - rect.width

        hoverEnabled: true
        drag.target: rect

        anchors {
            left: inputPorts.right
            right: outputPorts.left
            top: parent.top
            bottom: parent.bottom
        }

    }

}
