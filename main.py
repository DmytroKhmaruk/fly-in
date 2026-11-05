
import sys

from drone import Drone
from parser import MapParser
from pathfinder import Pathfinder
from scheduler import Scheduler
from simulation import Simulation


def main() -> None:
    if len(sys.argv) != 2:
        print("Usage: python3 main.py <map_file>")
        return

    try:
        parser = MapParser()

        graph, nb_drones, start, end = parser.parse(
            sys.argv[1]
        )

        pathfinder = Pathfinder(graph)
        scheduler = Scheduler(graph)

        paths = pathfinder.find_disjoint_paths(
            start, end
        )

        if not paths:
            raise ValueError(
                "No path connects start to end"
            )

        assignments = scheduler.distribute_drones(
            paths, nb_drones
        )

        drones: list[Drone] = []

        for path, drone_ids in zip(paths, assignments):
            for drone_id in drone_ids:
                drones.append(Drone(drone_id, path))

        simulation = Simulation(graph, drones)
        simulation.run()

    except (OSError, ValueError, RuntimeError) as error:
        print(f"Error: {error}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
