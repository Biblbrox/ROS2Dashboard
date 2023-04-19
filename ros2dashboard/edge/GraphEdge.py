from abc import ABC, abstractmethod

from std_msgs.msg import String

class GraphEdge(ABC):
    def __init__(self, node_name):
        self.node_name = node_name

    @abstractmethod
    def unique_key(self) -> str:
        pass

        
    

