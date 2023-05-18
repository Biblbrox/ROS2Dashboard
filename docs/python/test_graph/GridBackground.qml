import QtQuick 2.15

Item {
    id: root

    property real gridOpacity: 0.2
    property int gridSize: 20
    property int gridSizeMax: 60
    property int gridSizeMin: 10
    property string fillColor: "#464646"

    onGridSizeChanged: canvas.requestPaint()
    onGridOpacityChanged: canvas.requestPaint()

    Canvas {
        id: canvas

        anchors.fill: parent
        onPaint: {
            if (gridSize > gridSizeMax)
                gridSize = gridSizeMin;

            if (gridSize < gridSizeMin)
                gridSize = gridSizeMax;

            var ctx = getContext("2d");
            ctx.reset();
            /// Draw background
            ctx.fillStyle = fillColor;
            ctx.fillRect(0, 0, width, height);
            ctx.strokeStyle = "rgba(0, 0, 0, " + root.gridOpacity + ")";
            ctx.lineWidth = 1;
            ctx.beginPath();
            for (var x = 0; x < width; x += gridSize) {
                ctx.moveTo(x, 0);
                ctx.lineTo(x, height);
            }
            for (var y = 0; y < height; y += gridSize) {
                ctx.moveTo(0, y);
                ctx.lineTo(width, y);
            }
            ctx.stroke();
        }
    }

}
