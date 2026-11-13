
from drone import Drone
from graph import Graph
from pathfinder import Pathfinder
from scheduler import Scheduler
from simulation import Simulation


class RouteOptimizer:
    MAX_ALTERNATIVES = 4

    def __init__(self, graph: Graph) -> None:
        self.graph = graph
        self.pathfinder = Pathfinder(graph)
        self.scheduler = Scheduler(graph)

    def _make_drones(self, paths: list[list[str]], assignments: list[list[int]]
                     ) -> list[Drone]:
        drones = [
            Drone(drone_id, path)
            for path, ids in zip(paths, assignments)
            for drone_id in ids
        ]
        drones.sort(key=lambda drone: drone.drone_id)
        return drones

    def _simulate(self, drones: list[Drone], max_turns: int) -> int | None:
        try:
            simulation = Simulation(self.graph, drones)

            while (
                not all(d.finished for d in drones)
                and simulation.turn < max_turns
            ):
                simulation.step()

            if all(d.finished for d in drones):
                return simulation.turn

        except (ValueError, RuntimeError):
            return None

        return None

    def _alternatives(self, start: str, end: str, baseline: list[str]
                      ) -> list[list[str]]:
        found: set[tuple[str, ...]] = set()
        for first, second in zip(
            baseline,
            baseline[1:],
        ):
            edge = frozenset((first, second))
            path = self.pathfinder.dijkstra(start, end, excluded_edges={edge})
            if path and path != baseline:
                found.add(tuple(path))

        ordered = sorted(found, key=lambda path: (
                self.scheduler.path_cost(list(path)), path)
        )

        return [list(path) for path in ordered[:self.MAX_ALTERNATIVES]]

    def _split_drones(self, paths: list[list[str]], first_count: int,
                      total: int) -> list[Drone]:
        quotas = [first_count, total - first_count]
        assigned = [0, 0]
        drones: list[Drone] = []
        for drone_id in range(1, total + 1):
            choices = [i for i in range(2) if assigned[i] < quotas[i]]
            route_index = min(choices, key=lambda i: (
                              assigned[i] / quotas[i], i))
            drones.append(Drone(drone_id, paths[route_index]))
            assigned[route_index] += 1

        return drones

    def optimize(self, start: str, end: str, nb_drones: int,) -> list[Drone]:
        if nb_drones < 1:
            raise ValueError("Drone count must be positive")

        paths = self.pathfinder.find_disjoint_paths(start, end)
        if not paths:
            raise RuntimeError("No route found")

        assignments = self.scheduler.distribute_drones(paths, nb_drones)
        baseline_drones = self._make_drones(paths, assignments)
        if len(baseline_drones) != nb_drones:
            raise RuntimeError("Incomplete drone allocation")

        max_turns = max(100, nb_drones * len(self.graph.zones) * 5)
        best_turns = self._simulate(baseline_drones, max_turns)
        best_plan = [
            (drone.drone_id, drone.path.copy())
            for drone in baseline_drones
        ]
        primary = self.pathfinder.dijkstra(start, end)
        if not primary:
            raise RuntimeError("No route found")

        alternatives = self._alternatives(start, end, primary)
        for alternative in alternatives:
            pair = [primary, alternative]

            for first_count in range(nb_drones + 1):
                candidate = self._split_drones(pair, first_count, nb_drones)
                limit = (best_turns if best_turns is not None
                         else max_turns)
                turns = self._simulate(candidate, limit)
                if turns is None:
                    continue

                if (best_turns is None or turns < best_turns):
                    best_turns = turns
                    best_plan = [
                        (drone.drone_id, drone.path.copy())
                        for drone in candidate
                    ]

        if best_turns is None:
            raise RuntimeError("No successful simulation found")

        return [Drone(drone_id, path) for drone_id, path in best_plan]
