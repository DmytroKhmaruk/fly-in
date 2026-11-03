
from graph import Graph
from zone import Zone


def main() -> None:
    graph = Graph()

    graph.add_zone(Zone("Start", 0, 0))
    graph.add_zone(Zone("A", 3, 4))
    graph.add_zone(Zone("B", 5, 4))
    graph.add_zone(Zone("End", 10, 10))

    graph.add_connection("Start", "A")
    graph.add_connection("Start", "B")
    graph.add_connection("A", "End", 2)
    graph.add_connection("B", "End")

    for name, neighbors in graph.adjacency.items():
        print(f"{name}: {neighbors}")


if __name__ == "__main__":
    main()
