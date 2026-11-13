import heapq
from collections import deque
from graph import Graph


class Pathfinder:
    def __init__(self, graph: Graph) -> None:
        self.graph = graph

    def bfs(self, start: str, end: str) -> list[str]:
        if start not in self.graph.zones:
            raise ValueError(f"Unknown start zone: {start}")

        if end not in self.graph.zones:
            raise ValueError(f"Unknown end zone: {end}")

        if (
            self.graph.zones[start].zone_type == "blocked"
            or self.graph.zones[end].zone_type == "blocked"
        ):
            return []

        queue: deque[str] = deque([start])
        visited: set[str] = {start}
        parent: dict[str, str | None] = {start: None}
        while queue:
            current = queue.popleft()

            if current == end:
                return self._build_path(parent, end)

            for neighbor in self.graph.adjacency[current]:
                if neighbor in visited:
                    continue

                zone = self.graph.zones[neighbor]
                if zone.zone_type == "blocked":
                    continue

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

    def dijkstra(self, start: str, end: str, excluded: set[str] | None = None,
                 excluded_edges: set[frozenset[str]] | None = None
                 ) -> list[str]:
        if start not in self.graph.zones:
            raise ValueError(f"Unknown start zone: {start}")

        if end not in self.graph.zones:
            raise ValueError(f"Unknown end zone: {end}")

        excluded_zones = (excluded if excluded is not None else set())
        blocked_edges = (
            excluded_edges
            if excluded_edges is not None
            else set()
        )

        if start in excluded_zones or end in excluded_zones:
            return []

        if (
            self.graph.zones[start].zone_type == "blocked"
            or self.graph.zones[end].zone_type == "blocked"
        ):
            return []

        best: dict[str, tuple[int, int]] = {start: (0, 0)}
        parent: dict[str, str | None] = {start: None}
        heap: list[tuple[int, int, str]] = [(0, 0, start)]

        while heap:
            cost, negative_priority, current = (heapq.heappop(heap))

            if (cost, negative_priority) != best[current]:
                continue

            if current == end:
                return self._build_path(parent, end)

            for neighbor in self.graph.adjacency[current]:
                zone = self.graph.zones[neighbor]

                if (
                    neighbor in excluded_zones
                    or zone.zone_type == "blocked"
                ):
                    continue

                key = frozenset((current, neighbor))
                if key in blocked_edges:
                    continue

                new_cost = cost + zone.movement_cost()
                priority_bonus = (1 if zone.zone_type == "priority" else 0)
                new_priority = (negative_priority - priority_bonus)
                new_score = (new_cost, new_priority)

                if (neighbor not in best or new_score < best[neighbor]):
                    best[neighbor] = new_score
                    parent[neighbor] = current

                    heapq.heappush(heap, (new_cost, new_priority, neighbor))

        return []

    def find_disjoint_paths(self, start: str, end: str) -> list[list[str]]:
        if start == end:
            raise ValueError("Start and End must differ")

        paths: list[list[str]] = []
        excluded_zones: set[str] = set()
        excluded_edges: set[frozenset[str]] = set()

        while True:
            path = self.dijkstra(
                start, end, excluded=excluded_zones,
                excluded_edges=excluded_edges,
            )

            if not path:
                break

            paths.append(path)
            if len(path) == 2:
                excluded_edges.add(
                    frozenset((start, end))
                )
            else:
                excluded_zones.update(path[1:-1])

        if len(paths) == 1 and len(paths[0]) > 2:
            original = paths[0]

            original_edges = {
                frozenset((a, b))
                for a, b in zip(original, original[1:])
            }

            for first, second in zip(
                original,
                original[1:],
            ):
                excluded_edge = frozenset((first, second))
                alternative = self.dijkstra(
                    start, end, excluded_edges={excluded_edge}
                )
                if not alternative or alternative == original:
                    continue

                shared_zones = (set(original[1:-1]) & set(alternative[1:-1]))
                alternative_edges = {
                    frozenset((a, b))
                    for a, b in zip(alternative, alternative[1:],)
                }

                shared_edges = (original_edges & alternative_edges)
                zones_ok = all(
                    self.graph.zones[name].max_drones >= 2
                    for name in shared_zones
                )
                edges_ok = all(
                    self.graph.connections[key]
                    .max_link_capacity >= 2
                    for key in shared_edges
                )
                if zones_ok and edges_ok:
                    paths.append(alternative)
                    break

        return paths
