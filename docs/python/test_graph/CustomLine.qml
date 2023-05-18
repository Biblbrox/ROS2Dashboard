import QtQuick 2.0

Item {
    id: root

    property int sourceX: 0
    property int sourceY: 0
    property int targetX: 0
    property int targetY: 0

    function updateLine() {
        line.x1 = sourceX;
        line.y1 = sourceY;
        line.x2 = targetX;
        line.y2 = targetY;
    }

    onTargetXChanged: updateLine()
    onTargetYChanged: updateLine()

    Canvas {
        id: canvas

        anchors.fill: parent
        onPaint: {
            var ctx = getContext("2d");
            ctx.reset();
            ctx.beginPath();
            ctx.moveTo(line.x1, line.y1);
            ctx.lineTo(line.x2, line.y2);
            ctx.stroke();
        }
    }

    QtObject {
        id: line

        property int x1: sourceX
        property int y1: sourceY
        property int x2: targetX
        property int y2: targetY

        onX1Changed: canvas.requestPaint()
        onY1Changed: canvas.requestPaint()
        onX2Changed: canvas.requestPaint()
        onY2Changed: canvas.requestPaint()
    }

}
