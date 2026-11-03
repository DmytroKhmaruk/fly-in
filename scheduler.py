class Scheduler:
    def distribute_drones(self, paths: list[list[str]], nb_drones: int) -> list[list[int]]:
        if not paths:
            return []
        assignments: list[list[int]] = [[] for _ in paths]
        for drone_id in range(1,nb_drones + 1):
            path_index = (drone_id - 1) % len(paths)
            assignments[path_index].append(drone_id)
        return assignments