from threading import Thread
from rclpy.node import Node
import rclpy

class SpinThread:

    def __init__(self) -> None:
        self.threads = {}
        pass

    def add_node_execution(self, node: Node):
        if node.get_name() in self.threads.keys():
            self.threads[node.get_name()].join()

        self.threads[node.get_name()] = Thread(target=rclpy.spin, args=(node,))
        self.threads[node.get_name()].start()

