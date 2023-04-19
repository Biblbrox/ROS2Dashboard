import hashlib

from ros2dashboard.edge.GraphEdge import GraphEdge


class Host(GraphEdge):
    def __init__(self, ip: str, hostname: str) -> None:
        self.ip = ip
        self.hostname = hostname

    def unique_key(self) -> str:
        return hashlib.sha256(bytes(self.ip + self.hostname, 'utf-8')).hexdigest()
