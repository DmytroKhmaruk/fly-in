from collections import deque
from graph import Graph
import heapq


class Pathfinder:
    def __init__(self, graph: Graph) -> None:
        self.graph = graph

    def bfs(self, start: str, end: str) -> list[str]:
        if start not in self.graph.zones:
            raise ValueError(f"Unknowm start zone: {start}")
        if end not in self.graph.zones:
            raise ValueError(f"Unknown end zone: {end}")

        queue: deque[str] = deque([start])
        visited: set[str] = {start}
        parent: dict[str, str | None] = {start: None}
        while queue:
            current = queue.popleft()
            if current == end:
                return self._build_path(parent, end)

            for neighbor in self.graph.adjacency[current]:
                zone = self.graph.zones[neighbor]
                if zone.zone_type == "blocked":
                    continue
                if neighbor not in visited:
                    visited.add(neighbor)
                    parent[neighbor] = current
                    queue.append(neighbor)
        return []

    def _build_path(self, parent: dict[str, str | None], end: str
                    ) -> list[str]:
        path: list[str] = []
        current: str | None = end

        while current is not None:
            path.append(current)
            current = parent[current]
        path.reverse()
        return path

    def dijkstra(self, start: str, end: str, excluded: set[str] | None = None
                 ) -> list[str]:
        if start not in self.graph.zones:
            raise ValueError(f"Unknown start zone: {start}")
        if end not in self.graph.zones:
            raise ValueError(f"Unknown end zone: {end}")
        if excluded is None:
            excluded = set()
        if start in excluded or end in excluded:
            return []
        if self.graph.zones[start].zone_type == "blocked":
            return []
        if self.graph.zones[end].zone_type == "blocked":
            return []

        distances: dict[str, int] = {start: 0}
        parent: dict[str, str | None] = {start: None}

        heap: list[tuple[int, str]] = [(0, start)]
        while heap:
            current_cost, current = heapq.heappop(heap)
            if current_cost > distances[current]:
                continue
            if current == end:
                return self._build_path(parent, end)

            for neighbor in self.graph.adjacency[current]:
                zone = self.graph.zones[neighbor]
                if (neighbor in excluded or zone.zone_type == "blocked"):
                    continue

                new_cost = current_cost + zone.movement_cost()
                if (neighbor not in distances
                        or new_cost < distances[neighbor]):
                    distances[neighbor] = new_cost
                    parent[neighbor] = current
                    heapq.heappush(heap, (new_cost, neighbor))
        return []

    def find_disjoint_paths(self, start: str, end: str) -> list[list[str]]:
        paths: list[list[str]] = []
        excluded: set[str] = set()

        while True:
            path = self.dijkstra(start, end, excluded)
            if not path:
                break
            paths.append(path)
            for zone_name in path[1:-1]:
                excluded.add(zone_name)
            if len(path) == 2:
                break
        return paths
