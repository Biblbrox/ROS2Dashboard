import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import com.viz.types 1.0

RowLayout {

    RDButton {
        id: configureButton

        backgroundColor: Theme.node.color.configureButton
        icon.color: "transparent"
        iconSource: "Configure"
        width: 50

        onClicked: {
            Logger.debug("Configure node " + title.text);
            //daemonClientModel.configureNode(title.text);
        }
    }
    RDButton {
        id: activateButton

        backgroundColor: Theme.node.color.activateButton
        icon.color: "transparent"
        iconSource: "Launch"
        width: 50

        onClicked: {
            Logger.debug("Activate node " + title.text);
            // daemonClientModel.activateNode(title.text);
        }
    }
    RDButton {
        id: deactivateButton

        backgroundColor: Theme.node.color.deactivateButton
        icon.color: "transparent"
        iconSource: "Pause"
        width: 50

        onClicked: {
            Logger.debug("Deactivate node " + title.text);
            //daemonClientModel.deactivateNode(title.text);
        }
    }
    RDButton {
        id: cleaningUpButton

        backgroundColor: Theme.node.color.cleanUpButton
        icon.color: "transparent"
        iconSource: "CleaningUp"
        width: 50

        onClicked: {
            Logger.debug("Cleaning up node " + title.text);
            //daemonClientModel.cleaningUpNode(title.text);
        }
    }
    RDButton {
        id: shutdownButton

        backgroundColor: Theme.node.color.shutdownButton
        icon.color: "transparent"
        iconSource: "Close"
        width: 50

        onClicked: {
            Logger.debug("Kill node " + title.text);
            daemonClientModel.killNode(title.text);
        }
    }
}