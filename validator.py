from collections import Counter
from drone import Drone
from graph import Graph


class SimulationValidator:
    def __init__(self, graph: Graph) -> None:
        self.graph = graph

    def validate_state(self, drones: list[Drone], start: str, end: str
                       ) -> None:
        occupancy: Counter[str] = Counter()
        identifiers: set[int] = set()
        transit_links: Counter[frozenset[str]] = Counter()
        for drone in drones:
            if drone.drone_id in identifiers:
                raise ValueError("Duplicate drone ID")

            identifiers.add(drone.drone_id)
            if not 0 <= drone.position < len(drone.path):
                raise ValueError(f"Invalid position for D{drone.drone_id}")

            if drone.finished and drone.in_transit:
                raise ValueError(f"Invalid state for D{drone.drone_id}")

            if drone.in_transit:
                if drone.remaining_turns != 1:
                    raise ValueError("Invalid transit duration")

                next_index = drone.position + 1
                if next_index >= len(drone.path):
                    raise ValueError("Missing transit target")

                destination = drone.path[next_index]
                origin = drone.current_zone
                key = frozenset((origin, destination))
                if key not in self.graph.connections:
                    raise ValueError(
                        f"Unknown transit connection: "
                        f"{origin}-{destination}"
                    )

                transit_links[key] += 1
                if (self.graph.zones[destination].zone_type != "restricted"):
                    raise ValueError("Invalid transit target")
                continue
            if drone.finished:
                if drone.current_zone != end:
                    raise ValueError("Invalid delivered drone")
                continue

            occupancy[drone.current_zone] += 1
        for name, count in occupancy.items():
            if name in (start, end):
                continue

            capacity = self.graph.zones[name].max_drones
            if count > capacity:
                raise ValueError(f"Zone {name}: {count}/{capacity} drones")

        for key, count in transit_links.items():
            capacity = self.graph.connections[key].max_link_capacity
            if count > capacity:
                raise ValueError("Transit connection capacity exceeded")

    def validate_plan(self, drones: list[Drone], arriving: list[Drone],
                      direct: list[Drone], starting: list[Drone], start: str,
                      end: str) -> None:
        selected = arriving + direct + starting
        drone_ids = {id(drone) for drone in drones}
        for drone in selected:
            if id(drone) not in drone_ids:
                raise ValueError("Plan contains an unknown drone")

        arriving_ids = {id(drone) for drone in arriving}
        for drone in drones:
            if drone.in_transit:
                if id(drone) not in arriving_ids:
                    raise ValueError(f"D{drone.drone_id} must arrive this turn"
                                     )

        selected_ids = [d.drone_id for d in selected]
        if len(selected_ids) != len(set(selected_ids)):
            raise ValueError("Drone scheduled more than once")

        occupancy: Counter[str] = Counter()
        link_usage: Counter[frozenset[str]] = Counter()
        for drone in drones:
            if not drone.finished and not drone.in_transit:
                occupancy[drone.current_zone] += 1

        for drone in selected:
            next_index = drone.position + 1
            if next_index >= len(drone.path):
                raise ValueError("Movement beyond path")

            origin = drone.current_zone
            destination = drone.path[next_index]
            key = frozenset((origin, destination))
            if key not in self.graph.connections:
                raise ValueError("Missing connection")

            zone_type = self.graph.zones[destination].zone_type
            if drone in arriving:
                if not drone.in_transit:
                    raise ValueError("Invalid arrival")
                occupancy[destination] += 1

            elif drone in starting:
                if drone.in_transit:
                    raise ValueError("Already in transit")
                if zone_type != "restricted":
                    raise ValueError("Invalid transit start")
                occupancy[origin] -= 1
                link_usage[key] += 1

            else:
                if drone.in_transit:
                    raise ValueError("Invalid direct move")
                if zone_type in ("restricted", "blocked"):
                    raise ValueError("Invalid destination")
                occupancy[origin] -= 1
                occupancy[destination] += 1
                link_usage[key] += 1

        for name, count in occupancy.items():
            if name in (start, end):
                continue

            capacity = self.graph.zones[name].max_drones
            if count < 0 or count > capacity:
                raise ValueError(f"Zone capacity violation: {name}")

        for key, count in link_usage.items():
            capacity = self.graph.connections[key].max_link_capacity

            if count > capacity:
                raise ValueError("Connection capacity exceeded")
