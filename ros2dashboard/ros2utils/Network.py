import hashlib

from ros2dashboard.edge.GraphEdge import GraphEdge
from ros2dashboard.devices.Ros2Node import VISUALIZATION_NODE_PREFIX


class Host(GraphEdge):
    def __init__(self, ip: str, hostname: str) -> None:
        self.ip = ip
        self.hostname = hostname

    def unique_key(self) -> str:
        return hashlib.sha256(bytes(self.ip + self.hostname, 'utf-8')).hexdigest()


def filter_internal_edges(edges: list[GraphEdge]):
    if edges is None:
        return edges
    
    if len(edges) == 0:
        return edges
    
    if not hasattr(edges[0], 'node_name'):
        return edges

    filtered = filter(lambda edge: not edge.node_name.startswith(
        f"/{VISUALIZATION_NODE_PREFIX}"), edges)

    return list(filtered)
