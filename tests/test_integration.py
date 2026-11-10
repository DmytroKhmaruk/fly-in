
import unittest
from pathlib import Path

from drone import Drone
from parser import MapParser
from pathfinder import Pathfinder
from scheduler import Scheduler
from simulation import Simulation


class TestIntegration(unittest.TestCase):
    """Test the complete drone routing workflow."""

    def test_complete_map(self) -> None:
        """Deliver every drone using a map file."""
        root = Path(__file__).resolve().parent.parent
        map_file = root / "maps" / "test_map.txt"

        # 1. Parse the map.
        parser = MapParser()

        graph, count, start, end = parser.parse(
            str(map_file)
        )

        # 2. Find paths.
        pathfinder = Pathfinder(graph)
        paths = pathfinder.find_disjoint_paths(
            start, end
        )

        self.assertTrue(paths)

        # 3. Distribute drones.
        scheduler = Scheduler(graph)

        assignments = scheduler.distribute_drones(
            paths, count
        )

        drones: list[Drone] = []

        for path, drone_ids in zip(paths, assignments):
            for drone_id in drone_ids:
                drones.append(Drone(drone_id, path))

        self.assertEqual(len(drones), count)

        # 4. Run the simulation.
        simulation = Simulation(graph, drones)
        history: list[list[str]] = []

        max_turns = 100

        for _ in range(max_turns):
            if all(drone.finished for drone in drones):
                break

            movements = simulation.step()
            history.append(movements)

        # 5. Verify delivery and turn count.
        self.assertTrue(
            all(drone.finished for drone in drones),
            "Some drones were not delivered",
        )

        self.assertEqual(
            len(history),
            simulation.turn,
        )

        self.assertEqual(
            {drone.drone_id for drone in drones},
            set(range(1, count + 1)),
        )

        # Reference for our sample two-path map.
        self.assertEqual(simulation.turn, 4)


    def test_disconnected_graph(self) -> None:
        """Detect a graph with no start-to-end route."""
        from graph import Graph
        from zone import Zone

        graph = Graph()

        graph.add_zone(Zone("Start", 0, 0))
        graph.add_zone(Zone("A", 1, 0))
        graph.add_zone(Zone("End", 3, 0))

        graph.add_connection("Start", "A")

        pathfinder = Pathfinder(graph)

        paths = pathfinder.find_disjoint_paths(
            "Start", "End"
        )

        self.assertEqual(paths, [])


if __name__ == "__main__":
    unittest.main()
