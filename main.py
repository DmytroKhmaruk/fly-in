import sys
from gui import DroneGUI
from optimizer import RouteOptimizer
from parser import MapParser
from simulation import Simulation


def main() -> None:
    if len(sys.argv) != 2:
        print("Usage: python3 main.py <map_file>", file=sys.stderr)
        sys.exit(1)

    filename = sys.argv[1]
    try:
        graph, nb_drones, start, end = (MapParser().parse(filename))
        optimizer = RouteOptimizer(graph)
        drones = optimizer.optimize(start, end, nb_drones)
        simulation = Simulation(graph, drones)
        gui = DroneGUI(graph=graph, simulation=simulation, start=start,
                       end=end)
        gui.run()
    except (OSError, ValueError, RuntimeError) as error:
        print(f"Error: {error}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
