import rclpy
from rclpy.node import Node
from rclpy.lifecycle.node import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.executors import SingleThreadedExecutor

from std_msgs.msg import String


class MinimalPublisher(LifecycleNode):

    def __init__(self):
        super().__init__('test_node_string_publisher')
        self.publisher_ = self.create_publisher(String, 'minimal_publisher', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_configure() is called.")
        # return success
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_activate() is called.")
        self.timer.reset()
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_deactivate() is called.")
        self.timer.cancel()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("on_shutdown() is called.")
        self.timer.cancel()
        return TransitionCallbackReturn.SUCCESS

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    #setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    minimal_publisher = MinimalPublisher()

    executor = SingleThreadedExecutor()
    executor.add_node(minimal_publisher)
    executor.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
