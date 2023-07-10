import QtQuick

QtObject {
    id: iconObject

    property string source

    function getSource() {
        if (Theme.darkTheme) {
            return "qrc:///ui/icons/" + source + "Dark.svg";
        } else {
            return "qrc:///ui/icons/" + source + ".svg";
        }
    }
}