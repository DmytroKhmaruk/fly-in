class Drone:
    def __init__(self, drone_id: int, path: list[str],) -> None:
        if drone_id < 1:
            raise ValueError("Drone ID must be positive")

        if len(path) < 2:
            raise ValueError("Drone path must contain two zones")

        self.drone_id = drone_id
        self.path = path.copy()
        self.position = 0

        self.in_transit = False
        self.remaining_turns = 0
        self.finished = False

    @property
    def current_zone(self) -> str:
        return self.path[self.position]

    @property
    def next_zone(self) -> str | None:
        if self.finished or self.in_transit:
            return None

        next_index = self.position + 1
        if next_index >= len(self.path):
            return None

        return self.path[next_index]

    def start_transit(self) -> None:
        if self.finished or self.in_transit:
            raise ValueError("Drone cannot start transit")
        if self.position + 1 >= len(self.path):
            raise ValueError("No next zone")
        self.in_transit = True
        self.remaining_turns = 1

    def arrive(self) -> None:
        if self.finished:
            raise ValueError("Drone is already delivered")

        if self.position + 1 >= len(self.path):
            raise ValueError("No next zone")

        self.position += 1
        self.in_transit = False
        self.remaining_turns = 0

        if self.position == len(self.path) - 1:
            self.finished = True
