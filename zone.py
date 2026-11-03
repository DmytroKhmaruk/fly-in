class Zone:
    def __init__(self, name: str, x: int, y: int, zone_type: str = "normal", max_drones: int = 1,) -> None:
        self.name = name
        self.x = x
        self.y = y
        self.zone_type = zone_type
        self.max_drones = max_drones