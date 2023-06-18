pragma Singleton
import QtQuick

QtObject {
    property var darkTheme: true
    property QtObject font
    property QtObject background
    property QtObject node

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
            property color primary: darkTheme ? "black" : "white"
            property color secondary: darkTheme ? "white" : "black"
        }

    }

    node: QtObject {
        property QtObject color

        color: QtObject {
            property color title: darkTheme ? "#4b516b" : "#4b516b"
            property color body: darkTheme ? "#35384b" : "#9d9fb4"
            property color titleFont: darkTheme ? "#FFFFFF" : "#000000"
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
