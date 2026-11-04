
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
                if self.graph.zones[name].zone_type == "blocked":
                    raise ValueError(f"Blocked zone: {name}")

            for first, second in zip(drone.path, drone.path[1:]):
                key = frozenset((first, second))
                if key not in self.graph.connections:
                    raise ValueError(f"Missing connection: {first}-{second}")

    def _occupancy(self) -> dict[str, int]:
        counts: dict[str, int] = {}
        for drone in self.drones:
            if drone.finished or drone.in_transit:
                continue

            name = drone.current_zone
            counts[name] = counts.get(name, 0) + 1

        return counts

    def _plan_turn(self) -> tuple[list[Drone], list[Drone], list[Drone]]:
        occupancy = self._occupancy()
        arriving = [drone for drone in self.drones if drone.in_transit]
        leaving: dict[str, int] = {}
        entering: dict[str, int] = {}
        link_usage: dict[frozenset[str], int] = {}
        direct: list[Drone] = []
        starting: list[Drone] = []

        for drone in arriving:
            destination = drone.path[drone.position + 1]
            entering[destination] = (
                entering.get(destination, 0) + 1
            )

        ordered = sorted(self.drones, key=lambda drone: drone.position,
                         reverse=True)
        end = self.drones[0].path[-1] if self.drones else ""
        for drone in ordered:
            if drone.finished or drone.in_transit:
                continue

            destination = drone.next_zone
            if destination is None:
                continue

            origin = drone.current_zone
            key = frozenset((origin, destination))
            connection = self.graph.connections[key]
            used = link_usage.get(key, 0)
            if used >= connection.max_link_capacity:
                continue

            zone = self.graph.zones[destination]
            restricted = zone.zone_type == "restricted"
            if not restricted and destination != end:
                occupied = occupancy.get(destination, 0)
                outgoing = leaving.get(destination, 0)
                incoming = entering.get(destination, 0)
                available = (zone.max_drones - occupied + outgoing - incoming)
                if available <= 0:
                    continue

            link_usage[key] = used + 1
            leaving[origin] = leaving.get(origin, 0) + 1
            if restricted:
                starting.append(drone)
            else:
                direct.append(drone)
                entering[destination] = (entering.get(destination, 0) + 1)

        for drone in arriving:
            destination = drone.path[drone.position + 1]
            if destination == end:
                continue
            zone = self.graph.zones[destination]

            final_count = (
                occupancy.get(destination, 0)
                - leaving.get(destination, 0) + entering.get(destination, 0))
            if final_count > zone.max_drones:
                raise RuntimeError(f"Arrival conflict in zone {destination}")

        return arriving, direct, starting

    def step(self) -> list[str]:
        arriving, direct, starting = self._plan_turn()
        if not arriving and not direct and not starting:
            if not all(drone.finished for drone in self.drones):
                raise RuntimeError("Simulation is blocked")

        self.turn += 1
        movements: list[str] = []
        for drone in arriving:
            drone.arrive()
            movements.append(f"D{drone.drone_id}-{drone.current_zone}")

        for drone in direct:
            drone.arrive()
            movements.append(f"D{drone.drone_id}-{drone.current_zone}")

        for drone in starting:
            origin = drone.current_zone
            destination = drone.next_zone

            if destination is None:
                raise RuntimeError("Invalid transit destination")

            drone.start_transit()
            movements.append(f"D{drone.drone_id}-{origin}-{destination}")

        return movements

    def run(self) -> None:
        while not all(drone.finished for drone in self.drones):
            movements = self.step()
            if movements:
                print(" ".join(movements))
