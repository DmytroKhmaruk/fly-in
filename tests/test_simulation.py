
import unittest

from drone import Drone
from graph import Graph
from simulation import Simulation
from zone import Zone


class TestSimulation(unittest.TestCase):
    def test_normal_movement(self) -> None:
        graph = Graph()
        graph.add_zone(Zone("Start", 0, 0))
        graph.add_zone(Zone("End", 1, 0))
        graph.add_connection("Start", "End")

        drone = Drone(1, ["Start", "End"])
        simulation = Simulation(graph, [drone])

        moves = simulation.step()

        self.assertEqual(moves, ["D1-End"])
        self.assertEqual(simulation.turn, 1)
        self.assertTrue(drone.finished)

    def test_zone_capacity(self) -> None:
        graph = Graph()

        graph.add_zone(Zone("Start", 0, 0))
        graph.add_zone(Zone("A", 1, 0))
        graph.add_zone(Zone("End", 2, 0))

        graph.add_connection("Start", "A")
        graph.add_connection("A", "End")

        drones = [
            Drone(1, ["Start", "A", "End"]),
            Drone(2, ["Start", "A", "End"]),
        ]

        simulation = Simulation(graph, drones)

        simulation.step()

        at_a = sum(
            drone.current_zone == "A"
            for drone in drones
        )

        self.assertEqual(at_a, 1)

    def test_restricted_movement(self) -> None:
        graph = Graph()

        graph.add_zone(Zone("Start", 0, 0))
        graph.add_zone(Zone("R", 1, 0, "restricted"))
        graph.add_zone(Zone("End", 2, 0))

        graph.add_connection("Start", "R")
        graph.add_connection("R", "End")

        drone = Drone(1, ["Start", "R", "End"])
        simulation = Simulation(graph, [drone])

        first = simulation.step()

        self.assertEqual(first, ["D1-Start-R"])
        self.assertTrue(drone.in_transit)

        second = simulation.step()

        self.assertEqual(second, ["D1-R"])
        self.assertFalse(drone.in_transit)
        self.assertEqual(drone.current_zone, "R")

    def test_restricted_link_reuse(self) -> None:
        graph = Graph()
        graph.add_zone(Zone("Start", 0, 0))
        graph.add_zone(Zone("R", 1, 0, "restricted"))
        graph.add_zone(Zone("End", 2, 0))
        graph.add_connection("Start", "R", max_link_capacity=1)
        graph.add_connection("R", "End")
        drones = [Drone(1, ["Start", "R", "End"]),
                  Drone(2, ["Start", "R", "End"])]

        simulation = Simulation(graph, drones)
        first = simulation.step()
        second = simulation.step()
        self.assertEqual(first, ["D1-Start-R"])
        self.assertIn("D1-R", second)
        self.assertIn("D2-Start-R", second)
        self.assertTrue(drones[1].in_transit)

        third = simulation.step()
        self.assertIn("D1-End", third)
        self.assertIn("D2-R", third)

        fourth = simulation.step()
        self.assertEqual(fourth, ["D2-End"])
        self.assertTrue(all(d.finished for d in drones))
        self.assertEqual(simulation.turn, 4)

    def test_future_collision(self) -> None:
        graph = Graph()

        graph.add_zone(Zone("Start", 0, 0))
        graph.add_zone(Zone("A", 1, 1))
        graph.add_zone(Zone("B", 1, -1))
        graph.add_zone(
            Zone("R", 2, 0, "restricted")
        )
        graph.add_zone(Zone("End", 3, 0))

        graph.add_connection("Start", "A")
        graph.add_connection("Start", "B")
        graph.add_connection("A", "R")
        graph.add_connection("B", "R")
        graph.add_connection("R", "End")

        drones = [
            Drone(1, ["Start", "A", "R", "End"]),
            Drone(2, ["Start", "B", "R", "End"]),
        ]

        simulation = Simulation(graph, drones)

        simulation.step()
        simulation.step()

        in_transit = sum(
            drone.in_transit
            for drone in drones
        )

        self.assertLessEqual(in_transit, 1)


if __name__ == "__main__":
    unittest.main()
