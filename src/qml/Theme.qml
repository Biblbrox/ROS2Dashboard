pragma Singleton
import QtQuick

QtObject {
    property var darkTheme: true
    property QtObject font
    property QtObject button
    property QtObject background
    property QtObject node
    property QtObject panel
    property QtObject vizControls
    property QtObject vizArea
    property QtObject sideBar

    sideBar: QtObject {
        property QtObject color

        color: QtObject {
            property color background: darkTheme ? "#3e3949" : "#c0bfce"
        }
    }

    vizArea: QtObject {
        property QtObject color

        color: QtObject {
            property color background: darkTheme ? "#1d1d28" : "#eeedff"
            property color topicTitle: darkTheme ? "#2f2e40" : "#c0bfcd"
        }
    }

    vizControls: QtObject {
        property QtObject color

        color: QtObject {
            property color background: darkTheme ? "#3d3948" : "black"
        }
    }

    font: QtObject {
        property QtObject pointSize
        property QtObject color

        pointSize: QtObject {
            property int menu: 10
            property int normal: 16
            property int subTitle: 18
            property int title: 24
            property int nodeTitle: 14
        }

        color: QtObject {
            property color primary: darkTheme ? "white" : "black"
            property color secondary: darkTheme ? "black" : "white"
        }

    }

    panel: QtObject {
        property QtObject color

        color: QtObject {
            property color elementBackground: darkTheme ? "#3d3948" : "#c0b8a5"
            property color background: darkTheme ? "#2f2e40" : "#c0bfcd"
        }
    }

    button: QtObject {
        property QtObject color

        color: QtObject {
            //property color primary: darkTheme ? "white": "black"
            property color primary: "transparent"
            property color selected: darkTheme ? "#21232f" : "#808293"
            property color hovered: darkTheme ? "#35384b" : "#9d9fb4"
        }
    }

    node: QtObject {
        property QtObject color

        color: QtObject {
            property color title: darkTheme ? "#4b516b" : "#4b516b"
            property color body: darkTheme ? "#35384b" : "#9d9fb4"
            property color titleFont: darkTheme ? "#FFFFFF" : "#000000"
            property color edge: darkTheme ? "#FFFFFF" : "#000000"

            property color configureButton: darkTheme ? "#35384b" : "#9d9fb4"
            property color activateButton: darkTheme ? "#35384b" : "#9d9fb4"
            property color deactivateButton: darkTheme ? "#35384b" : "#9d9fb4"
            property color cleanUpButton: darkTheme ? "#35384b" : "#9d9fb4"
            property color shutdownButton: darkTheme ? "#35384b" : "#9d9fb4"
        }

    }

    background: QtObject {
        property QtObject color

        color: QtObject {
            property color primary: darkTheme ? "#4e4e4e" : "white"
            property color secondary: darkTheme ? "white" : "#333333"
            property color field: darkTheme ? "#1d1d27" : "#eeedff"
        }

    }

}
