import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Shapes
import QtQuick.Window
import com.viz.types 1.0

Window {
    id: settingsWindow

    signal currentGroupChanged(string group);

    color: Theme.background.color.field

    SplitView {
        id: splitView
        width: parent.width
        height: parent.height

        ListView {
            id: settingsGroups
            //width: parent.width
            SplitView.preferredWidth: 200
            height: parent.height

            model: settingsModel

            delegate: Rectangle {
                color: settingsGroups.currentIndex === index ? "lightsteelblue" : "white"
                height: 40
                width: settingsGroups.width

                signal groupSelected(string grp);

                Component.onCompleted: {
                    groupSelected.connect(settingsWindow.currentGroupChanged);
                }

                RowLayout {
                    Text {
                        text: group
                    }
                }

                MouseArea {
                    anchors.fill: parent

                    onClicked: {
                        settingsGroups.currentIndex = index;
                        Logger.debug(`Selected group ${group}`);
                        groupSelected(group);
                    }
                }
            }
        }


        ColumnLayout {
            id: settingsPerGroup
            SplitView.preferredWidth: parent.width - settingsGroups.width
            height: parent.height

            function changeParamGroup(group) {
                for (let i = settingsPerGroup.children.length; i > 0; i--) {
                    settingsPerGroup.children[i - 1].destroy()
                }
                Logger.debug("Loading params from group " + group);
                let paramsPerGroup = settingsModel.getGroupParams(group);

                for (let i = 0; i < paramsPerGroup.length; i++) {
                    let name = paramsPerGroup[i]['name'].toString();
                    let value = paramsPerGroup[i]['value'].toString();
                    Logger.debug("Param with name " + name + " and value " + value);
                    let paramElement = `
                        import QtQuick 
                        import QtQuick.Layouts
                        import QtQuick.Controls

                        RowLayout {
                            width: parent.width
                            height: parent.height
                            Text {
                                text: "${name}" 
                            }

                            Text {
                                text: "${value}"
                            }
                        }
                    `;

                    const newObject = Qt.createQmlObject(paramElement, settingsPerGroup, "paramElement");
                }
            }
        }

        Connections {
            target: settingsWindow

            function onCurrentGroupChanged(group) {
                Logger.debug(`Current group ${group}`);
                settingsPerGroup.changeParamGroup(group);
            }
        }
    }
}
