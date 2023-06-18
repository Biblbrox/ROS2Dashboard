import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtWebEngine

Rectangle {
    /*ListView {
        //model: packageModel

        id: nodeListModel

        delegate: Rectangle {
            Label {
                id: label

                x: 0
                y: 0
                //width: 578
                height: 100
                anchors.right: parent.right
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                font.pointSize: 21
                renderType: Text.QtRendering
                anchors.left: parent.left
                color: "#000000"
                text: packageModel.name
            }

        }

    }*/

    id: visualizarionWindow

    color: "#2f2e40"

    ColumnLayout {
        width: parent.width

        /*WebEngineView {
            //width: parent.width
            //color: "#2f2e40"

            //anchors.fill: parent
            //url: "https://doc.qt.io/qt-6/qtquickcontrols-index.html"
        }*/

        Rectangle {
            width: parent.width
            height: 30
            color: "#3d3948"
        }

    }

}
