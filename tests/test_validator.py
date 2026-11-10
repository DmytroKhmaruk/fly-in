
import unittest

from drone import Drone
from graph import Graph
from validator import SimulationValidator
from zone import Zone


class TestValidator(unittest.TestCase):
    """Test independent simulation validation."""

    def setUp(self) -> None:
        """Create a small test graph."""
        self.graph = Graph()

        self.graph.add_zone(Zone("Start", 0, 0))
        self.graph.add_zone(Zone("A", 1, 0))
        self.graph.add_zone(Zone("End", 2, 0))

        self.graph.add_connection("Start", "A")
        self.graph.add_connection("A", "End")

        self.validator = SimulationValidator(self.graph)

    def test_valid_state(self) -> None:
        """Accept valid drone positions."""
        drone = Drone(1, ["Start", "A", "End"])
        drone.arrive()

        self.validator.validate_state(
            [drone], "Start", "End"
        )

    def test_overloaded_zone(self) -> None:
        """Reject more drones than a zone can hold."""
        drones = [
            Drone(1, ["Start", "A", "End"]),
            Drone(2, ["Start", "A", "End"]),
        ]

        for drone in drones:
            drone.arrive()

        with self.assertRaises(ValueError):
            self.validator.validate_state(
                drones, "Start", "End"
            )

    def test_overloaded_link(self) -> None:
        """Reject simultaneous moves over link capacity."""
        drones = [
            Drone(1, ["Start", "A", "End"]),
            Drone(2, ["Start", "A", "End"]),
        ]

        with self.assertRaises(ValueError):
            self.validator.validate_plan(
                drones,
                arriving=[],
                direct=drones,
                starting=[],
                start="Start",
                end="End",
            )

    def test_valid_plan(self) -> None:
        """Accept one valid direct move."""
        drone = Drone(1, ["Start", "A", "End"])

        self.validator.validate_plan(
            [drone],
            arriving=[],
            direct=[drone],
            starting=[],
            start="Start",
            end="End",
        )


if __name__ == "__main__":
    unittest.main()
