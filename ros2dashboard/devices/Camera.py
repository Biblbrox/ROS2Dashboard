from ros2dashboard.devices.Ros2Node import GenericNode

class CameraNode(GenericNode):

    # unique node identifier domain.
    __identifier__ = 'ros2dashboard.devices.CameraNode'

    # initial default node name.
    NODE_NAME = 'Camera node'

    def __init__(self):
        super(CameraNode, self).__init__(self.NODE_NAME)

        # create an input port.
        self.add_input('in', color=(180, 80, 0))

        # create an output port.
        self.add_output('out')
