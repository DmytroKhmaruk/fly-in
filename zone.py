class Zone:
    VALID_TYPES = {"normal", "restricted", "priority", "blocked"}

    def __init__(self, name: str, x: int, y: int, zone_type: str = "normal",
                 max_drones: int = 1, color: str = "none") -> None:
        if zone_type not in self.VALID_TYPES:
            raise ValueError(f"Invalid zone type: {zone_type}")

        if max_drones < 1:
            raise ValueError("Zone capacity must be positive")

        self.name = name
        self.x = x
        self.y = y
        self.zone_type = zone_type
        self.max_drones = max_drones
        self.color = color

    def movement_cost(self) -> int:
        if self.zone_type == "restricted":
            return 2

        if self.zone_type == "blocked":
            raise ValueError("Cannot enter a blocked zone")

        return 1
