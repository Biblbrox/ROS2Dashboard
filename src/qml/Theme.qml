pragma Singleton
import QtQuick

QtObject {
    property var darkTheme: true
    property QtObject font
    property QtObject background
    property QtObject node
    property QtObject panel

    font: QtObject {
        property QtObject pointSize
        property QtObject color

        pointSize: QtObject {
            property int menu: 10
            property int normal: 12
            property int subTitle: 18
            property int title: 24
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

    node: QtObject {
        property QtObject color

        color: QtObject {
            property color title: darkTheme ? "#4b516b" : "#4b516b"
            property color body: darkTheme ? "#35384b" : "#9d9fb4"
            property color titleFont: darkTheme ? "#FFFFFF" : "#000000"
            property color edge: darkTheme ? "#FFFFFF" : "#000000"
        }

    }

    background: QtObject {
        property QtObject color

        color: QtObject {
            property color primary: darkTheme ? "#4e4e4e" : "white"
            property color secondary: darkTheme ? "white" : "#333333"
        }

    }

}
