
from graph import Graph
from pathfinder import Pathfinder
from zone import Zone


def main() -> None:
    graph = Graph()

    graph.add_zone(Zone("Start", 0, 0))
    graph.add_zone(Zone("R1", 1, 1, "restricted"))
    graph.add_zone(Zone("R2", 2, 1, "restricted"))
    graph.add_zone(Zone("A", 1, 2))
    graph.add_zone(Zone("B", 2, 2))
    graph.add_zone(Zone("C", 3, 2))
    graph.add_zone(Zone("End", 4, 1))

    graph.add_connection("Start", "R1")
    graph.add_connection("R1", "R2")
    graph.add_connection("R2", "End")

    graph.add_connection("Start", "A")
    graph.add_connection("A", "B")
    graph.add_connection("B", "C")
    graph.add_connection("C", "End")

    pathfinder = Pathfinder(graph)

    bfs_path = pathfinder.bfs("Start", "End")
    dijkstra_path = pathfinder.dijkstra("Start", "End")

    print("BFS:", " -> ".join(bfs_path))
    print("Dijkstra:", " -> ".join(dijkstra_path))


if __name__ == "__main__":
    main()
