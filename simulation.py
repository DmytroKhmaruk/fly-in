from drone import Drone
from graph import Graph

class Simulation:
    def __init__(self, graph: Graph, drones: list[Drone]) -> None:
        self.graph = graph
        self.drones = drones
        self.turn = 0
        self._validate_paths()

    def _validate_paths(self) -> None:
        for drone in self.drones:
            for name in drone.path:
                if name not in self.graph.zones:
                    raise ValueError(f"Unknown zone: {name}")

                zone = self.graph.zones[name]
                if zone.zone_type == "blocked":
                    raise ValueError(f"Blocked zone: {name}")
                if zone.zone_type == "restricted":
                    raise ValueError("Restricted zones are not supported yet")
            
            for first, second in zip(drone.path, drone.path[1:]):
                key = frozenset((first, second))
                if key not in self.graph.connections:
                    raise ValueError(f"Missing connection: {first}-{second}")

    def _occupancy(self) -> dict[str, int]:
        counts: dict[str, int] = {}
        for drone in self.drones:
            if drone.finished:
                continue

            name = drone.current_zone
            counts[name] = counts.get(name, 0) + 1

        return counts

    def _plan_turn(self) -> list[Drone]:
        occupancy = self._occupancy()
        leaving: dict[str, int] = {}
        entering: dict[str, int] = {}
        link_usage: dict[frozenset[str], int] = {}
        selected: list[Drone] = []
        ordered = sorted(self.drones, key=lambda drone: drone.position, reverse=True)

        for drone in ordered:
            if drone.finished or drone.in_transit:
                continue

            destination = drone.next_zone
            if destination is None:
                continue

            origin = drone.current_zone
            connection_key = frozenset((origin, destination))
            connection = self.graph.connections[connection_key]
            used = link_usage.get(connection_key, 0)
            if used >= connection.max_link_capacity:
                continue

            zone = self.graph.zones[destination]
            is_end = destination == drone.path[-1]
            if not is_end:
                occupied = occupancy.get(destination, 0)
                outgoing = leaving.get(destination, 0)
                incoming = entering.get(destination, 0)
                available = (zone.max_drones - occupied + outgoing - incoming)

                if available <= 0:
                    continue

            selected.append(drone)
            link_usage[connection_key] = used + 1
            leaving[origin] = leaving.get(origin, 0) + 1
            entering[destination] = (entering.get(destination, 0) + 1)

        return selected

    def step(self) -> list[str]:
        selected = self._plan_turn()
        if not selected and not all(drone.finished for drone in self.drones):
            raise RuntimeError("No progress possible: simulation is blocked")

        self.turn += 1
        movements: list[str] = []
        for drone in selected:
            drone.arrive()
            movements.append(f"D{drone.drone_id}-{drone.current_zone}")

        return movements

    def run(self) -> None:
        while not all(drone.finished for drone in self.drones):
            movements = self.step()
            if movements:
                print(" ".join(movements))