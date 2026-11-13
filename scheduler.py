from graph import Graph


class Scheduler:
    def __init__(self, graph: Graph) -> None:
        self.graph = graph

    def path_cost(self, path: list[str]) -> int:
        if len(path) < 2:
            raise ValueError("Path must contain at least two zones")

        cost = 0
        for name in path[1:]:
            zone = self.graph.zones[name]
            cost += zone.movement_cost()

        return cost

    def path_capacity(self, path: list[str]) -> int:
        if len(path) < 2:
            raise ValueError("Path must contain at least two zones")

        capacities: list[int] = []
        for first, second in zip(path, path[1:]):
            key = frozenset((first, second))
            connection = self.graph.connections[key]
            capacities.append(connection.max_link_capacity)

        for name in path[1:-1]:
            zone = self.graph.zones[name]
            capacities.append(zone.max_drones)

        return min(capacities)

    def distribute_drones(self, paths: list[list[str]], nb_drones: int
                          ) -> list[list[int]]:
        if nb_drones < 0:
            raise ValueError("Invalid number of drones")

        if not paths:
            return []

        assignments: list[list[int]] = [[] for _ in paths]
        costs = [self.path_cost(path) for path in paths]
        capacities = [self.path_capacity(path) for path in paths]
        for drone_id in range(1, nb_drones + 1):
            best_index = min(range(len(paths)), key=lambda i: (costs[i]
                             + len(assignments[i]) // capacities[i], i))
            assignments[best_index].append(drone_id)

        return assignments
