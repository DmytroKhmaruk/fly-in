
from drone import Drone
from graph import Graph
from pathfinder import Pathfinder
from scheduler import Scheduler
from simulation import Simulation
from zone import Zone


def main() -> None:
    graph = Graph()

    graph.add_zone(Zone("Start", 0, 0))
    graph.add_zone(Zone("A", 1, 1))
    graph.add_zone(Zone("B", 2, 1))
    graph.add_zone(Zone("C", 1, 2))
    graph.add_zone(Zone("D", 2, 2))
    graph.add_zone(Zone("End", 3, 1))

    graph.add_connection("Start", "A")
    graph.add_connection("A", "B")
    graph.add_connection("B", "End")

    graph.add_connection("Start", "C")
    graph.add_connection("C", "D")
    graph.add_connection("D", "End")

    pathfinder = Pathfinder(graph)
    scheduler = Scheduler()

    paths = pathfinder.find_disjoint_paths("Start", "End")

    if not paths:
        raise ValueError("No path from Start to End")

    assignments = scheduler.distribute_drones(paths, 5)

    drones: list[Drone] = []

    for path, drone_ids in zip(paths, assignments):
        for drone_id in drone_ids:
            drones.append(Drone(drone_id, path))

    simulation = Simulation(graph, drones)
    simulation.run()


if __name__ == "__main__":
    main()
