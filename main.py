
import sys

from drone import Drone
from gui import DroneGUI
from parser import MapParser
from pathfinder import Pathfinder
from scheduler import Scheduler
from simulation import Simulation


def main() -> None:

    if len(sys.argv) != 2:
        print(
            "Usage: python3 main.py <map_file>",
            file=sys.stderr,
        )
        sys.exit(1)

    filename = sys.argv[1]

    try:
        parser = MapParser()

        graph, nb_drones, start, end = parser.parse(
            filename
        )

        pathfinder = Pathfinder(graph)

        paths = pathfinder.find_disjoint_paths(
            start, end
        )

        if not paths:
            raise RuntimeError(
                "No route from Start to End"
            )

        scheduler = Scheduler(graph)

        assignments = scheduler.distribute_drones(
            paths, nb_drones
        )

        drones: list[Drone] = []

        for path, drone_ids in zip(
            paths, assignments
        ):
            for drone_id in drone_ids:
                drones.append(
                    Drone(drone_id, path)
                )

        simulation = Simulation(graph, drones)

        gui = DroneGUI(
            graph=graph,
            simulation=simulation,
            start=start,
            end=end,
        )

        gui.run()

    except (
        OSError,
        ValueError,
        RuntimeError,
    ) as error:
        print(
            f"Error: {error}",
            file=sys.stderr,
        )
        sys.exit(1)


if __name__ == "__main__":
    main()
