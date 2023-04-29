import pytest
from ros2dashboard.edge import Ros2Node, Publisher, Subscriber

def test_ros2_node():
    node = Ros2Node(node_name_="test_node", publishers_=[Publisher(topic_name="foo")], subscribers_=[Subscriber(topic_name="bar")])
    
    assert node.node_name == "test_node"
    assert len(node.publishers) == 1
    assert len(node.subscribers) == 1
    
    pub = node.get_publisher("nonexistent_topic")
    assert pub is None
    
    sub = node.get_subscription("bar")
    assert sub.topic_name == "bar"
    
    node.host = "127.0.0.1"
    
    node.clear_inputs()
    assert len(node.input_ports()) == 0
    
    key = node.unique_key()
    assert isinstance(key, str)