
import unittest

from graph import Graph
from scheduler import Scheduler
from zone import Zone


class TestScheduler(unittest.TestCase):
    def test_different_path_lengths(self) -> None:
        graph = Graph()

        graph.add_zone(Zone("Start", 0, 0))
        graph.add_zone(Zone("A", 1, 0))
        graph.add_zone(Zone("B", 1, 1))
        graph.add_zone(Zone("C", 2, 1))
        graph.add_zone(Zone("D", 3, 1))
        graph.add_zone(Zone("E", 4, 1))
        graph.add_zone(Zone("End", 5, 0))

        paths = [["Start", "A", "End"],
                 ["Start", "B", "C", "D", "E", "End"]]

        scheduler = Scheduler(graph)
        assignments = scheduler.distribute_drones(paths, 6)

        self.assertEqual(assignments, [[1, 2, 3, 4, 6], [5]])

    def test_restricted_cost(self) -> None:
        graph = Graph()

        graph.add_zone(Zone("Start", 0, 0))
        graph.add_zone(Zone("R", 1, 0, "restricted"))
        graph.add_zone(Zone("End", 2, 0))

        scheduler = Scheduler(graph)
        cost = scheduler.path_cost(["Start", "R", "End"])
        self.assertEqual(cost, 3)


if __name__ == "__main__":
    unittest.main()
