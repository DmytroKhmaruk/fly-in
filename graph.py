from zone import Zone
from connection import Connection


class Graph:
    def __init__(self) -> None:
        self.zones: dict[str, Zone] = {}
        self.adjacency: dict[str, list[str]] = {}
        self.connections: dict[frozenset[str], Connection] = {}

    def add_zone(self, zone: Zone) -> None:
        if zone.name in self.zones:
            raise ValueError(f"Duplicate zone: {zone.name}")
        self.zones[zone.name] = zone
        self.adjacency[zone.name] = []

    def add_connection(self, first: str, second: str,
                       max_link_capacity: int = 1) -> None:
        if first not in self.zones or second not in self.zones:
            raise ValueError("Connection refers to an unknown zone")
        if first == second:
            raise ValueError("Self-connections are not supported")
        if max_link_capacity < 1:
            raise ValueError("Connection capacity must be positive")

        key = frozenset((first, second))
        if key in self.connections:
            raise ValueError("Duplicate connection")

        connection = Connection(first, second, max_link_capacity,)
        self.connections[key] = connection
        self.adjacency[first].append(second)
        self.adjacency[second].append(first)
