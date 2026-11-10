
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

    
    def test_dependency_order(self) -> None:
        graph = Graph()

        for name, x in [
            ("Start", 0),
            ("X", 1),
            ("A", 2),
            ("B", 3),
            ("End", 4),
        ]:
            graph.add_zone(Zone(name, x, 0))

        graph.add_connection("Start", "X")
        graph.add_connection("X", "A")
        graph.add_connection("A", "B")
        graph.add_connection("Start", "B")
        graph.add_connection("B", "End")

        d1 = Drone(1, ["Start", "X", "A", "B", "End"])
        d2 = Drone(2, ["Start", "B", "End"])

        d1.arrive()
        d1.arrive()
        d2.arrive()

        simulation = Simulation(graph, [d1, d2])
        moves = simulation.step()

        self.assertIn("D1-B", moves)
        self.assertIn("D2-End", moves)

        self.assertEqual(d1.current_zone, "B")
        self.assertTrue(d2.finished)
        self.assertEqual(simulation.turn, 1)


    def test_circular_movement(self) -> None:
        """Allow a safe three-drone movement cycle."""
        graph = Graph()

        graph.add_zone(Zone("Start", 0, 0))
        graph.add_zone(Zone("A", 1, 1))
        graph.add_zone(Zone("B", 2, 1))
        graph.add_zone(Zone("C", 2, 2))
        graph.add_zone(Zone("End", 3, 1))

        for first, second in [
            ("Start", "A"),
            ("Start", "B"),
            ("Start", "C"),
            ("A", "B"),
            ("B", "C"),
            ("C", "A"),
            ("A", "End"),
            ("B", "End"),
            ("C", "End"),
        ]:
            graph.add_connection(first, second)

        d1 = Drone(1, ["Start", "A", "B", "End"])
        d2 = Drone(2, ["Start", "B", "C", "End"])
        d3 = Drone(3, ["Start", "C", "A", "End"])

        # Prepare one drone in each zone.
        d1.arrive()
        d2.arrive()
        d3.arrive()

        simulation = Simulation(graph, [d1, d2, d3])

        moves = simulation.step()

        self.assertCountEqual(
            moves,
            ["D1-B", "D2-C", "D3-A"],
        )

        self.assertEqual(d1.current_zone, "B")
        self.assertEqual(d2.current_zone, "C")
        self.assertEqual(d3.current_zone, "A")
        self.assertEqual(simulation.turn, 1)


    def test_swap_link_capacity(self) -> None:
        """Block opposite moves on a capacity-one link."""
        graph = Graph()

        graph.add_zone(Zone("Start", 0, 0))
        graph.add_zone(Zone("A", 1, 0))
        graph.add_zone(Zone("B", 2, 0))
        graph.add_zone(Zone("End", 3, 0))

        graph.add_connection("Start", "A")
        graph.add_connection("Start", "B")
        graph.add_connection("A", "B", 1)
        graph.add_connection("A", "End")
        graph.add_connection("B", "End")

        d1 = Drone(1, ["Start", "A", "B", "End"])
        d2 = Drone(2, ["Start", "B", "A", "End"])

        d1.arrive()
        d2.arrive()

        simulation = Simulation(graph, [d1, d2])

        with self.assertRaises(RuntimeError):
            simulation.step()


if __name__ == "__main__":
    unittest.main()
