
from drone import Drone
from graph import Graph
from validator import SimulationValidator


class Simulation:
    def __init__(self, graph: Graph, drones: list[Drone]) -> None:
        self.graph = graph
        self.drones = drones
        self.turn = 0
        self._validate_paths()
        self.validator = SimulationValidator(graph)

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

    def _guaranteed_departures(self, arriving: list[Drone], destination: str,
                               end: str) -> int:
        candidates = 0
        for drone in arriving:
            next_index = drone.position + 1
            if drone.path[next_index] != destination:
                continue

            after_index = next_index + 1
            if after_index >= len(drone.path):
                continue

            if drone.path[after_index] == end:
                candidates += 1

        if candidates == 0:
            return 0

        key = frozenset((destination, end))
        connection = self.graph.connections[key]
        return min(candidates, connection.max_link_capacity)

    def _plan_cycles(self, pending: list[Drone], occupancy: dict[str, int],
                     leaving: dict[str, int], entering: dict[str, int],
                     link_usage: dict[frozenset[str], int], direct: list[Drone]
                     ) -> None:
        candidates: dict[str, Drone] = {}
        for drone in pending:
            origin = drone.current_zone
            if origin == drone.path[0]:
                continue
            if occupancy.get(origin, 0) != 1:
                continue
            if leaving.get(origin, 0) != 0:
                continue
            if entering.get(origin, 0) != 0:
                continue

            candidates[origin] = drone
        scheduled: set[int] = set()
        for start in candidates:
            if candidates[start].drone_id in scheduled:
                continue

            seen: dict[str, int] = {}
            chain: list[Drone] = []
            cycle: list[Drone] = []
            current = start
            while current in candidates:
                if current in seen:
                    cycle = chain[seen[current]:]
                    break

                seen[current] = len(chain)
                drone = candidates[current]
                destination = drone.next_zone
                if destination is None:
                    break

                zone = self.graph.zones[destination]
                if zone.zone_type == "restricted":
                    break
                if destination not in candidates:
                    break

                chain.append(drone)
                current = destination
            else:
                cycle = []

            if not cycle:
                continue
            if len(cycle) < 2:
                continue

            usage: dict[frozenset[str], int] = {}
            valid = True
            for drone in cycle:
                destination = drone.next_zone
                if destination is None:
                    valid = False
                    break

                origin = drone.current_zone
                key = frozenset((origin, destination))
                connection = self.graph.connections[key]
                usage[key] = usage.get(key, 0) + 1
                total = (link_usage.get(key, 0) + usage[key])
                if total > connection.max_link_capacity:
                    valid = False
                    break
                if self.graph.zones[destination].max_drones < 1:
                    valid = False
                    break

            if not valid:
                continue

            for drone in cycle:
                destination = drone.next_zone
                if destination is None:
                    raise RuntimeError("Invalid cycle")

                origin = drone.current_zone
                key = frozenset((origin, destination))
                direct.append(drone)
                scheduled.add(drone.drone_id)
                leaving[origin] = leaving.get(origin, 0) + 1
                entering[destination] = (entering.get(destination, 0) + 1)
                link_usage[key] = (link_usage.get(key, 0) + 1)

    def _plan_turn(self) -> tuple[list[Drone], list[Drone], list[Drone]]:
        occupancy = self._occupancy()
        arriving = [drone for drone in self.drones if drone.in_transit]
        leaving: dict[str, int] = {}
        entering: dict[str, int] = {}
        link_usage: dict[frozenset[str], int] = {}
        reserved_next: dict[str, int] = {}
        direct: list[Drone] = []
        starting: list[Drone] = []
        end = self.drones[0].path[-1] if self.drones else ""
        start = self.drones[0].path[0] if self.drones else ""
        for drone in arriving:
            destination = drone.path[drone.position + 1]
            entering[destination] = (
                entering.get(destination, 0) + 1)

        pending = [
            drone for drone in self.drones
            if not drone.finished and not drone.in_transit]
        while pending:
            progress = False
            still_pending: list[Drone] = []

            for drone in pending:
                next_destination = drone.next_zone
                if next_destination is None:
                    continue

                destination = next_destination
                origin = drone.current_zone
                key = frozenset((origin, destination))
                connection = self.graph.connections[key]
                used = link_usage.get(key, 0)
                if used >= connection.max_link_capacity:
                    still_pending.append(drone)
                    continue

                zone = self.graph.zones[destination]
                restricted = zone.zone_type == "restricted"
                if restricted and destination != end:
                    occupied = occupancy.get(destination, 0)
                    outgoing = leaving.get(destination, 0)
                    arriving_now = entering.get(destination, 0)
                    reserved = reserved_next.get(destination, 0)
                    guaranteed = self._guaranteed_departures(
                        arriving, destination, end)

                    future_count = (
                        occupied - outgoing
                        + arriving_now - guaranteed + reserved)
                    if future_count >= zone.max_drones:
                        still_pending.append(drone)
                        continue

                elif destination != end:
                    occupied = occupancy.get(destination, 0)
                    outgoing = leaving.get(destination, 0)
                    incoming = entering.get(destination, 0)
                    available = (
                        zone.max_drones - occupied
                        + outgoing - incoming)
                    if available <= 0:
                        still_pending.append(drone)
                        continue

                link_usage[key] = used + 1
                leaving[origin] = leaving.get(origin, 0) + 1
                if restricted:
                    starting.append(drone)
                    reserved_next[destination] = (
                        reserved_next.get(destination, 0) + 1)
                else:
                    direct.append(drone)
                    entering[destination] = (entering.get(destination, 0) + 1)

                progress = True

            if not progress:
                break

            pending = still_pending

        self._plan_cycles(pending, occupancy, leaving, entering, link_usage,
                          direct)
        for name, zone in self.graph.zones.items():
            if name == start or name == end:
                continue

            final_count = (
                occupancy.get(name, 0)
                - leaving.get(name, 0)
                + entering.get(name, 0))
            if final_count > zone.max_drones:
                raise RuntimeError(f"Capacity exceeded in zone {name}")

        return arriving, direct, starting

    def step(self) -> list[str]:
        arriving, direct, starting = self._plan_turn()
        if not arriving and not direct and not starting:
            if not all(drone.finished for drone in self.drones):
                raise RuntimeError("Simulation is blocked")

        start = self.drones[0].path[0] if self.drones else ""
        end = self.drones[0].path[-1] if self.drones else ""

        self.validator.validate_plan(self.drones, arriving, direct, starting,
                                     start, end)
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

        self.validator.validate_state(self.drones, start, end)
        return movements

    def run(self) -> None:
        while not all(drone.finished for drone in self.drones):
            movements = self.step()
            if movements:
                print(" ".join(movements))
