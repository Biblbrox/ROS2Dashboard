import QtQuick 2.0
import QtMultimedia 5.4
import Ros2 1.0

Item {
  width: 256
  height: 144

  // property string nodeName

  ImageTransportSubscription {
    id: imageSubscriber
    topic: "/image_raw"
    //throttleRate: 0.2 // 1 frame every 5 seconds
    defaultTransport: "raw"
  }

  VideoOutput {
    source: imageSubscriber
    anchors.fill: parent
    // Can be used in increments of 90 to rotate the video
    orientation: 0
  }

  Component.onCompleted: {
    // Initialize ROS with the given name. The command line args are passed by the plugin
    // Optionally, you can call init with a string list ["arg1", "arg2"] after the name to use those
    // args instead of the ones supplied by the command line.
    Ros2.init(nodeName)
  }

}
