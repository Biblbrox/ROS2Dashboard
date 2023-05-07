from abc import ABC, abstractmethod

from ros2dashboard.edge.Topic import Topic
from ros2dashboard.edge.Client import Client
from ros2dashboard.edge.Subscriber import Subscriber
from ros2dashboard.edge.Publisher import Publisher
from ros2dashboard.edge.Service import Service
from ros2dashboard.edge.ActionClient import ActionClient
from ros2dashboard.edge.ActionServer import ActionServer
from ros2dashboard.edge.Package import Package
from ros2dashboard.ros2utils.Network import Host
from ros2dashboard.devices.Ros2Node import GenericNode

class Ros2Discover(ABC):
    def __init__(self) -> None:
        pass

    @abstractmethod
    def find_topics(self, node_name=None):
        pass

    @abstractmethod
    def find_subscribers(self, node_name=None) -> list[Subscriber]:
        pass
    
    @abstractmethod
    def find_publishers(self, node_name=None) -> list[Publisher]:
        pass

    @abstractmethod
    def find_service_servers(self, node_name=None) -> list[Service]:
        pass

    @abstractmethod
    def find_action_servers(self, node_name=None) -> list[ActionServer]:
        pass

    @abstractmethod
    def find_action_clients(self, node_name=None) -> list[ActionClient]:
        pass

    @abstractmethod
    def find_nodes(self) -> list[GenericNode]:
        pass

    @abstractmethod
    def find_node_names(self) -> list[str]:
        pass

    @abstractmethod
    def find_package_names(self) -> list[str]:
        pass

    @abstractmethod
    def find_packages(self) -> list[Package]:
        pass

    @abstractmethod
    def find_hosts(self) -> list[Host]:
        pass

    @abstractmethod
    def find_ros2_hosts(self):
        pass
