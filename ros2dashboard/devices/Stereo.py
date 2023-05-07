from ros2dashboard.devices.Ros2Node import GenericNode


class StereoNode(GenericNode):

    # unique node identifier domain.
    __identifier__ = 'ros2.stereo_camera'

    # initial default node name.
    NODE_NAME = 'Stereo camera node'

    def __init__(self):
        super(StereoNode, self).__init__()

        # create an input port.
        self.add_input('in', color=(180, 80, 0))

        # create an output port.
        self.add_output('out')
